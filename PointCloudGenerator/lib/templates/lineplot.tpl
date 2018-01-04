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

    <div id="plot"></div>
    <script>
        var data = [];
        var trace = {
        {% if ~isempty(legend) %} name: '{{legend}}',{% end %}
        x: [{{ strjoin(arrayfun(@(n) num2str(n),x,'UniformOutput',false),',')
 }}],
        y: [{{ strjoin(arrayfun(@(n) num2str(n),y,'UniformOutput',false),',')
 }}],
        type: "scatter"
        };
        data.push(trace);

        var layout = {
          {% if ~isempty(title) %} title: '{{title}}',{% end %}
          {% if ~isempty(xlabel) %} xaxis: { title: '{{xlabel}}' }, {% end %}
          {% if ~isempty(ylabel) %} yaxis: { title: '{{ylabel}}' }, {% end %}
        }

        Plotly.newPlot("plot", data, layout);
    </script>

    </body>
</html>