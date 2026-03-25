#!/usr/bin/env python3
"""
Dash map viewer for UM982 minimal CSV logs.

CSV columns expected:
  utc_time_iso,fix_type,lat_deg,lon_deg,true_heading_deg

Run:
  pip3 install pandas dash==2.* plotly
  python3 app.py --csv gnss_minimal_20260222_101500.csv

Then open http://127.0.0.1:8050/ in your browser.
"""

import argparse
import pandas as pd
from pathlib import Path

from dash import Dash, dcc, html, Input, Output, no_update
import plotly.express as px

# ---------- CLI ----------
parser = argparse.ArgumentParser(description="Dash map for GNSS CSV")
parser.add_argument("--csv", required=True, help="Path to CSV file from logger")
parser.add_argument("--port", type=int, default=8050, help="Dash port (default 8050)")
args = parser.parse_args()

csv_path = Path(args.csv)
if not csv_path.exists():
    raise SystemExit(f"CSV not found: {csv_path}")

# ---------- Load data ----------
df = pd.read_csv(csv_path)

# Normalize/clean columns
expected_cols = ["utc_time_iso", "fix_type", "lat_deg", "lon_deg", "true_heading_deg"]
missing = [c for c in expected_cols if c not in df.columns]
if missing:
    raise SystemExit(f"CSV is missing columns: {missing}")

# Drop rows without lat/lon
df = df.dropna(subset=["lat_deg", "lon_deg"]).copy()

# Optional: parse time for sorting/slider in future features
# df["utc_time_iso"] = pd.to_datetime(df["utc_time_iso"], errors="coerce")

# Define color map per fix type
# Add or adjust keys if your receiver emits other strings
FIX_COLORS = {
    "RTK Fixed": "#1f77b4",  # blue
    "RTK Float": "#ff7f0e",  # orange
    "DGPS":      "#2ca02c",  # green
    "GPS Fix":   "#d62728",  # red
    "No Fix":    "#7f7f7f",  # gray
    "Unknown":   "#9467bd"   # purple
}

# Provide color column that maps through above dict (fallback to Unknown)
df["fix_color"] = df["fix_type"].map(lambda s: FIX_COLORS.get(str(s), FIX_COLORS["Unknown"]))

# A small hover-friendly label
df["hover"] = (
    "UTC: " + df["utc_time_iso"].astype(str) +
    "<br>Fix: " + df["fix_type"].astype(str) +
    "<br>Lat: " + df["lat_deg"].map(lambda v: f"{v:.7f}") +
    "<br>Lon: " + df["lon_deg"].map(lambda v: f"{v:.7f}") +
    "<br>Heading: " + df["true_heading_deg"].map(lambda v: "-" if pd.isna(v) else f"{v:.1f}°")
)

# ---------- Dash app ----------
app = Dash(__name__)
app.title = "GNSS Map"

fix_types_sorted = sorted(df["fix_type"].dropna().unique().tolist())

app.layout = html.Div(
    style={"fontFamily": "Segoe UI, Roboto, Arial", "margin": "10px"},
    children=[
        html.H3("GNSS Fix Viewer"),
        html.Div(
            style={"display": "flex", "gap": "1rem", "alignItems": "center", "flexWrap": "wrap"},
            children=[
                html.Div([
                    html.Label("Fix types"),
                    dcc.Dropdown(
                        id="fix-filter",
                        options=[{"label": ft, "value": ft} for ft in fix_types_sorted],
                        value=fix_types_sorted,      # show all at start
                        multi=True,
                        clearable=False,
                        style={"minWidth": "260px"}
                    )
                ]),
                html.Div([
                    html.Label("Point size"),
                    dcc.Slider(id="size-slider", min=4, max=14, step=1, value=8,
                               marks={i:str(i) for i in range(4, 15, 2)},
                               tooltip={"placement": "bottom"})
                ], style={"minWidth": "220px", "flex": "1 1 220px"}),
            ]
        ),
        dcc.Graph(id="map", style={"height": "80vh"}),
        html.Div(
            f"Loaded {len(df):,} points from {csv_path.name}",
            style={"color": "#555", "marginTop": "6px"}
        )
    ]
)

@app.callback(
    Output("map", "figure"),
    Input("fix-filter", "value"),
    Input("size-slider", "value"),
)
def update_map(selected_fixes, point_size):
    if not selected_fixes:
        return no_update

    dff = df[df["fix_type"].isin(selected_fixes)]

    # Build a color discrete map only for present categories to keep the legend neat
    present = sorted(dff["fix_type"].dropna().unique().tolist())
    color_discrete_map = {ft: FIX_COLORS.get(ft, FIX_COLORS["Unknown"]) for ft in present}

    # Use Plotly Express scatter_mapbox with OpenStreetMap (no token needed)
    fig = px.scatter_mapbox(
        dff,
        lat="lat_deg",
        lon="lon_deg",
        color="fix_type",
        hover_name="fix_type",
        hover_data={"lat_deg": False, "lon_deg": False, "hover": True, "fix_type": False},
        custom_data=["hover"],
        color_discrete_map=color_discrete_map,
        zoom=12,                 # try to auto-zoom later if needed
        height=700
    )

    # Use OpenStreetMap tiles (no Mapbox token required)
    fig.update_layout(
        mapbox_style="open-street-map",
        margin=dict(l=0, r=0, t=10, b=0),
        legend_title_text="Fix type",
    )

    # Point size & marker style
    fig.update_traces(
        marker=dict(size=point_size, opacity=0.9),
        hovertemplate="%{customdata[0]}<extra></extra>"
    )

    # Auto-center map to data
    if not dff.empty:
        lat_center = dff["lat_deg"].mean()
        lon_center = dff["lon_deg"].mean()
        fig.update_layout(mapbox_center={"lat": lat_center, "lon": lon_center})

    return fig

if __name__ == "__main__":
    app.run(debug=True, port=args.port)