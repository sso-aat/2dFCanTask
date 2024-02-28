docs:
	dot -Tsvg remi-2dFCanTask.gv > remi-2dFCanTask.svg
	qr --factory=svg "http://aatlij:8081/" > qr.svg
	markdown remi-2dFCanTask.md > remi-2dFCanTask.md.html
	wkhtmltopdf --enable-local-file-access --page-size a4 --minimum-font-size 19 --user-style-sheet style.css remi-2dFCanTask.md.html remi-2dFCanTask.md.pdf

clean:
	rm -f remi-2dFCanTask.svg qr.svg remi-2dFCanTask.md.html remi-2dFCanTask.md.pdf
