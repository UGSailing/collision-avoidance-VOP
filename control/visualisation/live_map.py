import argparse
import os
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

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
                overflow: hidden; 
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
    
    dcc.Interval(
        id='interval-updater',
        interval=2000, 
        n_intervals=0
    )
])

@app.callback(
    Output('live-map', 'figure'),
    Input('interval-updater', 'n_intervals')
)
def update_map(n):
    try:
        df = pd.read_csv(os.path.join(DATA_FOLDER, 'points.csv'))
        path_df = pd.read_csv(os.path.join(DATA_FOLDER, 'path.csv'))
    except Exception as _:
        return dash.no_update

    # 1. Create the base map
    # We separate obstacles from other points to apply the hitbox only to them
    obstacles_df = df[df['category'] == 'gps']
    other_points_df = df[df['category'] != 'gps']

    fig = go.Figure()

    # 2. Add the HITBOX layer (Semi-transparent red areas)
    # This represents the 5.0m safety zone around obstacles
    fig.add_trace(go.Scattermap(
        lat=obstacles_df["latitude"],
        lon=obstacles_df["longitude"],
        mode="markers",
        marker=dict(
            size=35,           # Large size to simulate the 5m area visually
            color="red",
            opacity=0.3        # Transparent so overlaps are visible
        ),
        name="hitbox",
        hoverinfo="skip"
    ))

    # 3. Add the actual points (Obstacles and Destinations)
    # Using px logic for the categorical coloring
    for cat in df['category'].unique():
        cat_df = df[df['category'] == cat]
        color = "blue" if cat == "gps" else "green"
        
        fig.add_trace(go.Scattermap(
            lat=cat_df["latitude"],
            lon=cat_df["longitude"],
            mode="markers",
            marker=dict(size=12, color=color),
            name=cat,
            customdata=cat_df[["category"]],
            hovertemplate="%{customdata[0]}<extra></extra>"
        ))

    # 4. Overlay the path line
    fig.add_trace(go.Scattermap(
            lat=path_df["latitude"],
            lon=path_df["longitude"],
            mode="lines",
            line=dict(width=3, color="red"),
            hoverinfo="skip",
            name="path"
        ))

    fig.update_layout(
        mapbox_style="open-street-map",
        margin={"r":0,"t":0,"l":0,"b":0},
        showlegend=False,
        uirevision='constant',
        mapbox=dict(
            center=dict(lat=df["latitude"].mean(), lon=df["longitude"].mean()),
            zoom=18 # Closer zoom to see the hitboxes clearly
        )
    )
    
    return fig

if __name__ == '__main__':
    app.run(debug=True)