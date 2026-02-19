import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import pandas as pd
import plotly.express as px

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
        interval=2000, 
        n_intervals=0
    )
])

# this function is called every time the interval ticks
@app.callback(
    Output('live-map', 'figure'),
    Input('interval-updater', 'n_intervals')
)
def update_map(n):
    # load CSV
    try:
        df = pd.read_csv('mapping/points.csv')
    except Exception as e:
        # failsafe in case the CSV is currently being written to by another process
        return dash.no_update

    # 2. create the map with Plotly
    fig = px.scatter_map(
        df, 
        lat="latitude", 
        lon="longitude", 
        color="category", # differentiates dots with different color per category
        hover_data={"category": True, "latitude": False, "longitude": False},
        zoom=15
    )

    fig.update_layout(
        mapbox_style="open-street-map",
        margin={"r":0,"t":0,"l":0,"b":0}, # fullscreen
        showlegend=False,
        uirevision='constant'             # prevents the map from resetting zoom/pan on live update
    )
    
    fig.update_traces(
        marker=dict(size=12, opacity=0.8),
        hovertemplate="%{customdata[0]}<extra></extra>"
    )

    return fig

if __name__ == '__main__':
    # debug=True allows for hot-reloading if you change the Python code
    app.run(debug=True)