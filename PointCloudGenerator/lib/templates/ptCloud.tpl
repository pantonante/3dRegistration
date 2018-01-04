<!doctype html>
<html>
    <head>
        <meta charset="utf-8">
        <meta http-equiv="x-ua-compatible" content="ie=edge">
        <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
        <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    </head>
    <body>
        <!--[if lte IE 9]>
            <p class="browserupgrade">You are using an <strong>outdated</strong> browser. Please <a href="https://browsehappy.com/">upgrade your browser</a> to improve your experience and security.</p>
        <![endif]-->

<div id="ptCloud"></div>
<script>

Plotly.d3.csv('{{ptCloud}}', function(err, rows){
function unpack(rows, key) {
    return rows.map(function(row){
        return row[key]; }
    );
}

var ptCloud = {
    x:unpack(rows, 'x'), y: unpack(rows, 'y'), z: unpack(rows, 'z'),
    mode: 'markers',
    marker: {
        size: 1,
        line: {
            color: 'rgba(217, 217, 217, 1)',
            width: 0
        },
        opacity: 1
    },
    type: 'scatter3d'
};

var layout = {
  margin: { t: 30, l: 0, r: 0, b: 30 },
  showlegend: false, 
  autosize: true
};

Plotly.newPlot('ptCloud', [ptCloud], layout);
});

</script>


    </body>
</html>