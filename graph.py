import plotly.express as px
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import pandas

app = dash.Dash(__name__) #
app.layout = html.Div(
    html.Div([
        html.H4('Vex V5'),
        html.Div(id='live-update-text'),
        dcc.Graph(id='live-update-graph'),
        dcc.Interval(
            id='interval-component',
            interval=500, # in milliseconds
            n_intervals=0
        )
    ])
)

@app.callback(Output('live-update-graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_graph_live(n):
    df = pandas.read_csv('test.csv')
    plot = px.scatter(df, x="X", y=["Y", "theta"], height=800)
    # plot.uirevision = True
    return plot

if __name__ == '__main__':
    app.run_server(debug=True)