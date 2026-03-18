import argparse
import logging
import os
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('folder', type=str, help='Folder where points.csv and path.csv are stored')
args = parser.parse_args()
DATA_FOLDER = args.folder

# init
app = dash.Dash(__name__)

# CSS for clean design
app.index_string = '''
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            body {
                margin: 0;
                padding: 0;
                background-color: black;
                overflow: hidden; /* Prevent scrollbars */
            }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>
'''

app.layout = html.Div([
    dcc.Graph(id='live-map', style={'height': '100vh', 'width': '100vw', 'border': 'none'}),
    
    # the interval component triggers a function call automatically
    dcc.Interval(
        id='interval-updater',
        interval=500, 
        n_intervals=0
    )
])

# this function is called every time the interval ticks
@app.callback(
    Output('live-map', 'figure'),
    Input('interval-updater', 'n_intervals')
)
def update_map(n):
    # load CSVs
    try:
        df = pd.read_csv(os.path.join(DATA_FOLDER, 'points.csv'))
        path_df = pd.read_csv(os.path.join(DATA_FOLDER, 'path.csv'))
    except Exception as _:
        # failsafe in case the CSVs are currently being written to by another process
        return dash.no_update

    # create the map with Plotly
    fig = px.scatter_map(
        df, 
        lat="latitude", 
        lon="longitude", 
        color="category", # differentiates dots with different color per category
        hover_data={"category": True, "latitude": False, "longitude": False},
        zoom=15
    )

    # overlay the path as a line trace
    fig.add_trace(go.Scattermap(
            lat=path_df["latitude"],
            lon=path_df["longitude"],
            mode="lines",
            line=dict(width=3, color="red"),
            hoverinfo="skip",
            name="path"
        ))


    # --- ADD THIS GEOFENCE TRACE ---
    if hasattr(config, 'GEOFENCE_POND_ZWIJNAARDE') and config.GEOFENCE_POND_ZWIJNAARDE:
        # Extract Lat/Lon lists
        geo_lats = [pt[0] for pt in config.GEOFENCE_POND_ZWIJNAARDE]
        geo_lons = [pt[1] for pt in config.GEOFENCE_POND_ZWIJNAARDE]
        
        # Close the polygon by adding the first point to the end
        geo_lats.append(geo_lats[0])
        geo_lons.append(geo_lons[0])

        fig.add_trace(go.Scattermap(
            lat=geo_lats,
            lon=geo_lons,
            mode="lines",
            fill="toself", 
            fillcolor="rgba(0, 0, 255, 0.15)", 
            line=dict(width=3, color="blue"), # <--- FIXED
            hoverinfo="skip",
            name="geofence"
        ))

    # --- 2. ADD EXCLUSION ZONES (KEEP-OUT) ---
    if hasattr(config, 'EXCLUSION_ZONES') and config.EXCLUSION_ZONES:
        for i, zone in enumerate(config.EXCLUSION_ZONES):
            # Extract Lat/Lon for this specific exclusion polygon
            ex_lats = [pt[0] for pt in zone]
            ex_lons = [pt[1] for pt in zone]
            
            # Close the polygon
            ex_lats.append(ex_lats[0])
            ex_lons.append(ex_lons[0])

            fig.add_trace(go.Scattermap(
                lat=ex_lats,
                lon=ex_lons,
                mode="lines",
                fill="toself", 
                fillcolor="rgba(255, 0, 0, 0.3)", # Reddish for danger zones
                line=dict(width=2, color="red"),
                hoverinfo="all",
                name=f"Exclusion Zone {i+1}"
            ))
    # --------------------------------
        

    fig.update_layout(
        mapbox_style="open-street-map",
        margin={"r":0,"t":0,"l":0,"b":0}, # fullscreen
        showlegend=False,
        uirevision='constant'             # prevents the map from resetting zoom/pan on live update
    )
    
    fig.update_traces(
        selector=dict(type="scattermap", mode="markers"),
        marker=dict(size=12, opacity=0.8),
        hovertemplate="%{customdata[0]}<extra></extra>"
    )

    return fig

if __name__ == '__main__':
    logging.getLogger('werkzeug').setLevel(logging.CRITICAL) # only log critical errors to avoid cluttering the console
    app.run(debug=False) # debug=True allows for hot-reloading if you change the Python code