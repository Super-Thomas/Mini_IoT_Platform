<html>
    <head>
        <title>Mini IoT Platform Web Chart</title>
<?php
    // connect to mysql
    $db = mysqli_connect('Your MySQL IP', 'Your MySQL Account ID', 'Your MySQL Account Password', 'Your DB Name'); 

    // get score for dev0
    $sql = "select distinct DATE_FORMAT(a.regtime, '%m-%d %H:%i:%s') as regtime, a.score as dev1, b.score as dev2 from \
        (select * from sensor_table where id = 'dev0' order by regtime desc limit 2048) as a JOIN \
        (select * from sensor_table where id = 'dev1' order by regtime desc limit 2048) as b ON a.regtime = b.regtime order by regtime;";
    $result = mysqli_query($db, $sql);
    while ($row = mysqli_fetch_assoc($result)) {
        $data_array_score_dev0[] = ($row);
    }
    // convert to json
    $chart_score_dev0 = json_encode($data_array_score_dev0);

    // get temp for dev0
    $sql = "select distinct DATE_FORMAT(a.regtime, '%m-%d %H:%i:%s') as regtime, a.temp as dev1, b.temp as dev2 from \
        (select * from sensor_table where id = 'dev0' order by regtime desc limit 2048) as a JOIN \
        (select * from sensor_table where id = 'dev1' order by regtime desc limit 2048) as b ON a.regtime = b.regtime order by regtime;";
    $result = mysqli_query($db, $sql);
    while ($row = mysqli_fetch_assoc($result))
    {
        $data_array_temp_dev0[] = ($row);
    }
    // convert to json
    $chart_temp_dev0 = json_encode($data_array_temp_dev0);
    //echo $chart_temp_dev0;

    // get hum for dev0
    $sql = "select distinct DATE_FORMAT(a.regtime, '%m-%d %H:%i:%s') as regtime, a.hum as dev1, b.hum as dev2 from \
        (select * from sensor_table where id = 'dev0' order by regtime desc limit 2048) as a JOIN \
        (select * from sensor_table where id = 'dev1' order by regtime desc limit 2048) as b ON a.regtime = b.regtime order by regtime;";
    $result = mysqli_query($db, $sql);
    while ($row = mysqli_fetch_assoc($result))
    {
        $data_array_hum_dev0[] = ($row);
    }
    // convert to json
    $chart_hum_dev0 = json_encode($data_array_hum_dev0);
?>

<script src="http://ajax.googleapis.com/ajax/libs/jquery/1.3.1/jquery.min.js"></script>
<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
<!-- -->
    <script type="text/javascript">
        google.charts.load('current', {'packages': ['corechart']});
        google.charts.setOnLoadCallback(drawVisualization);

        function drawVisualization() {
            var chart_array = <?php echo $chart_score_dev0; ?>; // data for chart
            var header = ['regtime', 'dev1', 'dev2']; // header for chart
            var row = "";
            var rows = new Array();
            jQuery.each(chart_array, function (i, d) {
                // make row
                row = [
                    d.regtime,
		    parseInt(d.dev1),
		    parseInt(d.dev2)
                ];
                rows.push(row); // push data 
            });
            var jsonData = [header].concat(rows);
            var data = google.visualization.arrayToDataTable(jsonData);
            var options = {
                title: 'Air-score Table',
                vAxis: { title: 'Score' },
                hAxis: { title: 'Time' },
		        curveType: 'function'
            };
            var chart = new google.visualization.LineChart(document.getElementById('chartOfScore'));
            chart.draw(data, options);
        }
     </script>
<!-- -->
<!-- -->
<script type="text/javascript">
        google.charts.load('current', {'packages': ['corechart']});
        google.charts.setOnLoadCallback(drawVisualization);

        function drawVisualization() {
            var chart_array = <?php echo $chart_temp_dev0; ?>; // data for chart
            var header = ['regtime', 'dev1', 'dev2']; // header for chart
            var row = "";
            var rows = new Array();
            jQuery.each(chart_array, function (i, d) {
                // make row
                row = [
                    d.regtime,
			        parseFloat(d.dev1),
				parseFloat(d.dev2)
                ];
                rows.push(row); // push data 
            });
            var jsonData = [header].concat(rows);
            var data = google.visualization.arrayToDataTable(jsonData);
            var options = {
                title: 'Temprature Table',
                vAxis: { title: 'Temprature' },
                hAxis: { title: 'Time' },
		        curveType: 'function'
            };
            var chart = new google.visualization.LineChart(document.getElementById('chartOfTemp'));
            chart.draw(data, options);
        }
     </script>
<!-- -->
<!-- -->
<script type="text/javascript">
        google.charts.load('current', {'packages': ['corechart']});
        google.charts.setOnLoadCallback(drawVisualization);

        function drawVisualization() {
            var chart_array = <?php echo $chart_hum_dev0; ?>; // data for chart
            var header = ['regtime', 'dev1', 'dev2']; // header for chart
            var row = "";
            var rows = new Array();
            jQuery.each(chart_array, function (i, d) {
                // make row
                row = [
                    d.regtime,
			        parseFloat(d.dev1),
				parseFloat(d.dev2)
                ];
                rows.push(row); // push data 
            });
            var jsonData = [header].concat(rows);
            var data = google.visualization.arrayToDataTable(jsonData);
            var options = {
                title: 'Humidity Table',
                vAxis: { title: 'Humidity (%)' },
                hAxis: { title: 'Time' },
		        curveType: 'function'
            };
            var chart = new google.visualization.LineChart(document.getElementById('chartOfHum'));
            chart.draw(data, options);
        }
     </script>
<!-- -->
    </head>
    <body>
	    <div id="chartOfScore" style="width: 100%; height: 500px;"></div>
        <div id="chartOfTemp" style="width: 100%; height: 500px;"></div>
	    <div id="chartOfHum" style="width: 100%; height: 500px;"></div>
    </body>
</html>
