Zum Ausführen und Kompilieren werden folgende Libs benötigt:
* OpenCV	https://opencv.org/
* Pistache	http://pistache.io/
* ZBar		http://zbar.sourceforge.net/download.html


Parameter zum Ausführen:
-o: ouput.
    0 = kein Output (standard)
    1 = Video anzeigen (nicht auf dem Pi, da kein Bildschirm angeschlossen / keine GUI installiert)
    2 = Video speichern
	3 = Frames per Webserver (pistache) als jpeg zur Verfügung stellen
	4 = Input-Video speichern (ohne Linien einzuzeichnen)

-c: camera.
    ID der Kamera. Bei nur einer angeschlossenen Kamera immer 0

-v: Videopfad
    z. B. ~/Videos/Test.avi