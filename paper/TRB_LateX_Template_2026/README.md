# Paper LaTeX Template

This folder contains an updated LaTeX template for Annual Meeting papers. It was
generated from the previous LaTeX template and aligned with the supplied paper
requirements and Word template.

The template is unofficial. Authors are responsible for checking the current
venue instructions before submission.

## Included Files

- `trb_template.tex`: sample manuscript using the required title page,
  structured abstract, body sections, declarations, and references.
- `trbunofficial.cls`: class file for page layout, fonts, line numbering,
  title-page metadata, captions, and citations.
- `trb.bst`: BibTeX style adjusted for author-date references, full first
  names in the reference list, and DOI output.
- `trb_template.bib`: example bibliography.
- `trb_template-gumbel.pdf`: example figure used by the sample manuscript.
- `latexmkrc`: lightweight latexmk configuration.

## Key Requirements Reflected

- Letter page size with 1 inch margins.
- Editable Times-style font size in `trb_template.tex`; the default is `10pt`.
  Change the `10pt` option in `\documentclass[10pt,numbered]{...}` to `11pt`
  or `12pt` only if the target venue allows it. Leave `\manuscriptclass` empty
  to auto-check the document class file, or specify a class name without the
  `.cls` extension when multiple class files are available. With XeLaTeX or
  LuaLaTeX, the class uses Times New Roman when it is installed; with pdfLaTeX,
  it uses the Times-compatible `newtx` fonts.
- Single-column, single-spaced layout.
- Line numbers that restart at 1 on each page when the `numbered` class option
  is used. Template spacing uses `\numberedblankline` when a visible blank line
  should also receive a line number.
- Italic running header text with page numbers centered at the bottom of each
  page. The running header is calculated automatically from the author names.
- Title page with title, author names, job titles, affiliations, emails,
  optional ORCID IDs, editable total page count, submission date, and
  corresponding author marker.
- Structured abstract on its own page with the required headings:
  Objectives, Methods, Findings, Novelty, and Practical Applications.
- Author-date citations such as `\citep{smith2020}` for `(Smith 2020)` and
  `\citep[25--27]{smith2021}` for `(Smith 2021, 25--27)`.
- References sorted alphabetically by author through BibTeX.
- Bibliography style and database selection can be automatic. Leave
  `\bibliographystyle{}` and `\bibliography{}` empty to check for files that
  match the manuscript name first, then fall back to `trb.bst` and
  `trb_template.bib`. Enter names without file extensions to override the
  detected files.
- No appendices or supplemental material. The class raises an error if
  `\appendix` is used.
- Acknowledgments comments for the venue's AI disclosure policy.

## Build

From this folder:

```powershell
latexmk -pdf trb_template.tex
```

The title page includes an editable `\totalpages{...}` line in
`trb_template.tex`. Leave the braces empty to calculate the page count
automatically, enter a number to override it, or comment out that line if the
total page count is not needed.

If `latexmk` is unavailable, run:

```powershell
pdflatex trb_template.tex
bibtex trb_template
pdflatex trb_template.tex
pdflatex trb_template.tex
```

No `-shell-escape` flag is required because this version does not run
`texcount`.

## Template Commands

Use this author command:

```latex
\author*{Name}{Job title}{Institutional affiliation}{Email}[Address][ORCID]
```

Use `\author*` for the corresponding author and `\author` for all other authors.
Address and ORCID are optional.

The running header is generated from author last names. For example, three
authors produce `Name, Name, and Name`. Use `\AuthorHeaders{...}` only when a
manual override is needed.

Use `\submissiondate{}` to print the current date. Use
`\submissiondate{May 29, 2026}` to print a manually specified date, or comment
out the line if the submitted-date line is not needed. Use `\totalpages{}` to
print the automatically calculated page count, `\totalpages{5}` to print a
manually specified page count, or comment out the line if it is not needed.

Write the structured abstract directly in the `.tex` file so each required
heading is visible and easy to edit:

```latex
\section*{ABSTRACT}
\begingroup
\setlength{\parindent}{0pt}
\numberedblankline
\noindent\textbf{Objectives:} Objectives paragraph.
\par\numberedblankline
\noindent\textbf{Methods:} Methods paragraph.
\par\numberedblankline
\noindent\textbf{Findings:} Findings paragraph.
\par\numberedblankline
\noindent\textbf{Novelty:} Novelty paragraph.
\par\numberedblankline
\noindent\textbf{Practical Applications:} Practical Applications paragraph.
\par\endgroup
\newpage
```

Keep the structured abstract to 300 words or fewer and do not include figures,
tables, equations, or undefined acronyms.
