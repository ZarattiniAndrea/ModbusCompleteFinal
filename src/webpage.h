#ifndef WEBPAGE_H // Significa: “Se non è stato definito il simbolo WEBPAGE_H, allora procedi con il codice seguente”.
//#ifndef serve come protezione contro le inclusioni multiple, evitando definizioni duplicate.
#define WEBPAGE_H
#include <Arduino.h>

String generateHTML(int debounceDelay){
    String html = "<!DOCTYPE html><html><head><title>Arduino Web Server</title></head><body>";
    html += "<p>\n</p>";
    html += "<p>Valore di Debounce attualmente impostato a: " + String(debounceDelay) + "ms </p>";
    html += "<input type='button' value='Aggiorna' onClick='window.location.reload();' />"; // Pulsante per aggiornare la pagina
    html += "<form method='GET' action='/'><label for='debounce'>Imposta nuovo valore di Debounce (ms):</label>"; // campo di input
    html += "<input type='number' id='debounce' name='debounce' min='0' max='10000' required>";
    html += "<input type='submit' value='Invia'></form>";
    html += "</body></html>";
    return html;
}


#endif // WEBPAGE_H