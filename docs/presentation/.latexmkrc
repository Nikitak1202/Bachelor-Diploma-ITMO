# latexmk config for Beamer presentation (pdflatex).
# Build is run from this directory so ../images resolves correctly.
$pdf_mode = 1;
$pdflatex = 'pdflatex -interaction=nonstopmode -file-line-error -synctex=1 %O %S';
@default_files = ('main.tex');
