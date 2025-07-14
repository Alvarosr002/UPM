t <html><head><title>AD Input</title>
t <script language=JavaScript type="text/javascript" src="xml_http.js"></script>
t <script language=JavaScript type="text/javascript">
# Define URL and refresh timeout
t var formUpdate = new periodicObj("rtm.cgx", 1000);
t function plotRTMeasures() {
t  lumVal = parseFloat(document.getElementById("lum_value").value);
t  humVal = parseFloat(document.getElementById("hum_value").value);
t  AirQualVal = parseInt(document.getElementById("AQ_value").value);
t  WatLevVal = parseFloat(document.getElementById("WL_value").value);
t  tempVal = parseFloat(document.getElementById("temp_value").value);
t  document.getElementById("temp_value").value = tempVal.toFixed(1) + ' °C';
t  document.getElementById("hum_value").value = humVal.toFixed(1) + ' %';
t  document.getElementById("lum_value").value = lumVal.toFixed(1) + ' lux';
t  document.getElementById("AQ_value").value = AirQualVal.toFixed(1) + ' PM';
t  document.getElementById("WL_value").value = WatLevVal.toFixed(1) + ' %';
t }
t function periodicUpdateRTM() {
t   updateMultiple(formUpdate,plotRTMeasures);
t   setTimeout(periodicUpdateRTM, formUpdate.period);
t }
t var formUpdate1 = new periodicObj("botton.cgx", 500);
t function plotADGraph2() {
t  cont = document.getElementById("contador").value;
t }
t periodicUpdateRTM();
t </script></head>
i pg_header.inc
t <h2 align="center" style="color:#228B22"><br>Medidas en Tiempo Real</h2>
t <form action="rtm.cgi" method="post" name="rtm">
t <input type="hidden" value="rtm" name="pg">
t <table border=0 width=99%>
t <tr style="background-color: #228B22">
t  <th width=15% style="color:#FFFFFF">Sensor</th>
t  <th width=15% style="color:#FFFFFF">Medida</th>
t </tr>
t <tr><td>Temperatura:</td>
t   <td align="center">
t <input type="text" readonly style="background-color: transparent; border: 0px"
c b 1 size="10" id="temp_value" value="%5.1f °C"></td>
t </tr>
t <tr><td>Humedad:</td>
t   <td align="center">
t <input type="text" readonly style="background-color: transparent; border: 0px"
c b 2 size="10" id="hum_value" value="%5.1f lux"></td>
t </tr>
t <tr><td>Luminosidad:</td>
t   <td align="center">
t <input type="text" readonly style="background-color: transparent; border: 0px"
c b 3 size="10" id="lum_value" value="%5.1f %"></td>
t </tr>
t <tr><td>Calidad del aire:</td>
t   <td align="center">
t <input type="text" readonly style="background-color: transparent; border: 0px"
c b 4 size="10" id="AQ_value" value="%d PM"></td>
t </tr>
t </tr>
t <tr><td>Nivel del agua depósito:</td>
t   <td align="center">
t <input type="text" readonly style="background-color: transparent; border: 0px"
c b 5 size="10" id="WL_value" value="%5.1f %"></td>
t </tr>
t </table>
t <p align=left>
t <input type=submit name=lowMode1 value="Bajo Consumo" id="sbm1" onClick="window.location.href='/index.htm'"></p>
t <p align=center>
t <input type="button" value="Ver Consumo" id="sbm1" onClick="window.location.href='/consumption.cgi'"></p>
t <p align=right>
t <input type=submit name=exitLow value="Salir BC" id="sbm1" onClick="window.location.href='/index.htm'"></p>
t </form>
i pg_footer.inc
. End of script must be closed with period
