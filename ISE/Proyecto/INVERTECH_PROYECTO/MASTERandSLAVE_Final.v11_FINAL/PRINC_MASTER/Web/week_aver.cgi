t <html><head><title>Registro de Medidas</title>
t <script language=JavaScript type="text/javascript" src="xml_http.js"></script>
t <script language=JavaScript type="text/javascript">
# Define URL and refresh timeout
t var formUpdate = new periodicObj("week_aver.cgx", 25000);
t function plotWeekAverage() {
t  media = document.getElementById("average").value;
t  document.getElementById("tablaMedidas").innerHTML = media;
t }
t function periodicUpdateWA() {
t   updateMultiple(formUpdate,plotWeekAverage);
t   setTimeout(periodicUpdateWA, formUpdate.period);
t }
t periodicUpdateWA();
t </script></head>
i pg_header.inc
t <h2 align="center" style="color:#228B22"><br>Registro del Invernadero</h2>
t <form action="week_aver.cgi" method="post" name="wka">
t <input type="hidden" value="media" name="pg">
t <table border=0 width=99% id="tablaMedidas">
t <tr style="background-color: #228B22">
t  <th width=90% style="color:#FFFFFF">Histórico de medidas</th></tr>
t <p align=center>
t <input type=hidden readonly size="8"
c k 1 id="media" value="%s"></p>
t </table>
#t <p align=center>
#t <input type=submit name=RefMeas value="Actualizar" id="sbm2" onClick="periodicUpdateWA()"></p>
t </form>
i pg_footer.inc
. End of script must be closed with period
