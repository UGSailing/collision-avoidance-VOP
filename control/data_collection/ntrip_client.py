import asyncio
import base64
import time
import config
import env

async def run_ntrip_client(rtcm_queue: asyncio.Queue, get_latest_gga_raw):
    """
    Runs an NTRIP client that connects to a caster, sends GGA (if available),
    and puts received RTCM data into the provided queue.
    
    Args:
        rtcm_queue: asyncio.Queue to put received RTCM data into
        get_latest_gga_raw: callable that returns the latest raw GGA string (or None)
    """
    if not config.NTRIP_ENABLED:
        return

    print(f"Starting NTRIP client ({config.NTRIP_HOST}:{config.NTRIP_PORT})...")
    
    auth = base64.b64encode(f"{env.NTRIP_USER}:{env.NTRIP_PASSWORD}".encode("ascii")).decode("ascii") if env.NTRIP_USER else None
    path = f"/{config.NTRIP_MOUNT.lstrip('/')}"
    
    headers = [
        f"GET {path} HTTP/1.0",
        f"Host: {config.NTRIP_HOST}",
        "User-Agent: NTRIP pyclient-async/1.0",
        "Accept: */*",
        "Connection: close",
        "Ntrip-Version: Ntrip/2.0",
    ]
    if auth:
        headers.append(f"Authorization: Basic {auth}")
    request = ("\r\n".join(headers) + "\r\n\r\n").encode("ascii")

    while True:
        try:
            reader, writer = await asyncio.open_connection(
                config.NTRIP_HOST, config.NTRIP_PORT, ssl=config.NTRIP_USE_SSL
            )
            
            writer.write(request)
            await writer.drain()

            # Read headers
            buff = b""
            while b"\r\n\r\n" not in buff:
                try:
                    chunk = await asyncio.wait_for(reader.read(4096), timeout=10.0)
                except asyncio.TimeoutError:
                    raise RuntimeError("NTRIP: timeout waiting for headers")
                    
                if not chunk:
                    raise RuntimeError("NTRIP: connection closed during headers")
                buff += chunk
            
            head, rest = buff.split(b"\r\n\r\n", 1)
            head_text = head.decode("iso-8859-1", errors="ignore")
            
            # Use 'ICY 200' check for some casters
            if "200" not in head_text and "ICY 200" not in head_text:
                print(f"NTRIP Error: {head_text.splitlines()[0]}")
                writer.close()
                await writer.wait_closed()
                await asyncio.sleep(5)
                continue

            print(f"NTRIP Connected to {config.NTRIP_MOUNT}")
            
            if rest:
                if not rtcm_queue.full():
                    rtcm_queue.put_nowait(rest)

            prev_gga_send = 0
            
            while True:
                # Send periodic GGA to keep connection alive / receive virtual reference
                now = time.monotonic()
                if now - prev_gga_send >= config.NTRIP_SEND_GGA_EVERY:
                    latest_gga = get_latest_gga_raw()
                    if latest_gga:
                        try:
                            # ensure CRLF
                            msg = latest_gga.strip() + "\r\n"
                            writer.write(msg.encode("ascii", errors="ignore"))
                            await writer.drain()
                            prev_gga_send = now
                        except Exception:
                            # broken pipe or similar
                            break

                # Read RTCM chunk with a timeout to allow the loop to check GGA logic
                try:
                    chunk = await asyncio.wait_for(reader.read(4096), timeout=1.0)
                    if not chunk:
                        break # Connection closed
                    if not rtcm_queue.full():
                        rtcm_queue.put_nowait(chunk)
                except asyncio.TimeoutError:
                    continue 

            print("NTRIP Stream ended.")
            writer.close()
            await writer.wait_closed()

        except asyncio.CancelledError:
            print("NTRIP Client stopped.")
            # If writer exists, close it
            try:
                writer.close() # pyright: ignore[reportPossiblyUnboundVariable]
                await writer.wait_closed() # pyright: ignore[reportPossiblyUnboundVariable]
            except:
                pass
            raise
        except Exception as e:
            print(f"NTRIP Reconnecting in 3s: {e}")
            await asyncio.sleep(3)
