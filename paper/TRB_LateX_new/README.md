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
  Change `\documentclass[10pt,numbered]{trbunofficial}` to `11pt` or `12pt`
  only if the target venue allows it. With XeLaTeX or LuaLaTeX, the class uses
  Times New Roman when it is installed; with pdfLaTeX, it uses the
  Times-compatible `newtx` fonts.
- Single-column, single-spaced layout.
- Line numbers that restart at 1 on each page when the `numbered` class option
  is used.
- Italic running header text with page numbers centered at the bottom of each
  page. The running header is calculated automatically from the author names.
- Title page with title, author names, job titles, affiliations, emails,
  optional ORCID IDs, automatically calculated total page count, submission
  date, and corresponding author marker.
- Structured abstract on its own page with the required headings:
  Objectives, Methods, Findings, Novelty, and Practical Applications.
- Author-date citations such as `\citep{smith2020}` for `(Smith 2020)` and
  `\citep[25--27]{smith2021}` for `(Smith 2021, 25--27)`.
- References sorted alphabetically by author through BibTeX.
- No appendices or supplemental material. The class raises an error if
  `\appendix` is used.
- Acknowledgments comments for the venue's AI disclosure policy.

## Build

From this folder:

```powershell
latexmk -pdf trb_template.tex
```

The title page calculates total pages automatically with `LastPage`. Run at
least two passes so the total page count can resolve; `latexmk` handles those
passes automatically.

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

The submitted date is calculated automatically from `\today`. Add
`\submissiondate{...}` only when a manual date override is needed. Total pages
are calculated automatically.

Write the structured abstract directly in the `.tex` file so each required
heading is visible and easy to edit:

```latex
\section*{ABSTRACT}
\begingroup
\setlength{\parindent}{0pt}
\vspace{\baselineskip}

\noindent\textbf{Objectives:} Objectives paragraph.
\par\vspace{\baselineskip}

\noindent\textbf{Methods:} Methods paragraph.
\par\vspace{\baselineskip}

\noindent\textbf{Findings:} Findings paragraph.
\par\vspace{\baselineskip}

\noindent\textbf{Novelty:} Novelty paragraph.
\par\vspace{\baselineskip}

\noindent\textbf{Practical Applications:} Practical Applications paragraph.
\par\endgroup
\newpage
```

Keep the structured abstract to 300 words or fewer and do not include figures,
tables, equations, or undefined acronyms.
