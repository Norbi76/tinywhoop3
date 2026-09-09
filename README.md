Adresa repository: https://github.com/Norbi76/tinywhoop3.git

Pentru a putea compila proiectele este necesara instalarea ESP IDF v6.0.2 !!
Fiecare dintre cele 2 proiecte se va compila si scrie pe micro separat.
Mediul de dezvolatare ESP IDF a fost utilizat pe Linux(Ubuntu).

Pasii pentru compilare si scriere: 
1. cd ~/.espressif/v6.0.2/esp-idf
2. . ./export.sh
3. cd tinywhoop3/flight_controller/ sau cd tinywhoop3/telemetry_module/ 
4. idf.py set-target esp32s3
5. idf.py build flash monitor