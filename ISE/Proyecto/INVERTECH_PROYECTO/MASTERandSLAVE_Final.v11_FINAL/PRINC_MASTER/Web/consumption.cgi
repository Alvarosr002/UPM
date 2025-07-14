t <html><head><title>Consumo</title>
t <script language=JavaScript type="text/javascript" src="xml_http.js"></script>
t <script language=JavaScript type="text/javascript">
# Define URL and refresh timeout
t var formUpdate = new periodicObj("consumption.cgx", 1000);
t function plotConsumption() {
t  consumVal = parseFloat(document.getElementById("consum_amps").value);
t  document.getElementById("consum_amps").value = (consumVal.toFixed(3) + 'mA');
t }
t function periodicUpdateConsumption() {
t   updateMultiple(formUpdate,plotConsumption);
t   setTimeout(periodicUpdateConsumption, formUpdate.period);
t }
t periodicUpdateConsumption();
t </script></head>
i pg_header.inc
t <h2 align="center" style="color:#228B22"><br>Consumo Actual del Sistema</h2>
t <form action="consumption.cgi" method="post" name="ad">
t <input type="hidden" value="ad" name="pg">
t <table border=0 width=99%><font size="3">
t <tr style="background-color: #228B22">
#t  <th width=15% style="color:#FFFFFF">Item</th>
t  <th width=15% style="color:#FFFFFF">Amps</th>
#t  <th width=55%>Bargraph</th></tr>
#t <tr><td><img src="pabb.gif">Consumo Actual:</td>
t <td align="center"><input type="text" readonly style="background-color: transparent; color:#FFFFFF; border: 0px"
c e 1  size="10" id="consum_amps" value="%5.1f mA"></td>
#t <td height=50><table bgcolor="#FFFFFF" border="2" cellpadding="0" cellspacing="0" width="100%"><tr>
#c e 2 <td><table id="consum_table" style="width: %d%%" border="0" cellpadding="0" cellspacing="0">
#t <tr><td bgcolor="#0000FF">&nbsp;</td></tr></table></td></tr></table></td></tr>
t </font></table>
t <p align=center>
t <input type=submit name=lowMode1 value="Bajo Consumo" id="sbm1" onClick="window.location.href='/index.htm'"></p>
t <p align=center>
t <input type="button" value="Ver Medidas" id="sbm1" onClick="window.location.href='/rtm.cgi'"></p>
t </form>
i pg_footer.inc
. End of script must be closed with period
