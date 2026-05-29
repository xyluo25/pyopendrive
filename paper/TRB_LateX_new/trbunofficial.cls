% ======================================================================
% Annual Meeting paper template
% Class: trbunofficial.cls
% Updated for the local paper requirements and supplied Word template.
% ======================================================================

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{trbunofficial}[2026/05/29 Annual Meeting paper template]

% ----------------------------------------------------------------------
% Class options
%   numbered -- turn on line numbers that restart on each page.
%   10pt, 11pt, 12pt -- document font size; default is 10pt.
% ----------------------------------------------------------------------
\newif\ifpaper@numbered
\paper@numberedfalse
\newcommand*\paper@fontsize{10pt}
\DeclareOption{numbered}{\paper@numberedtrue}
\DeclareOption{10pt}{\renewcommand*\paper@fontsize{10pt}}
\DeclareOption{11pt}{\renewcommand*\paper@fontsize{11pt}}
\DeclareOption{12pt}{\renewcommand*\paper@fontsize{12pt}}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\PassOptionsToClass{\paper@fontsize}{article}
\LoadClass[titlepage,oneside]{article}
\PassOptionsToPackage{fleqn}{amsmath}

% ----------------------------------------------------------------------
% Packages
% ----------------------------------------------------------------------
\RequirePackage{iftex}
\ifPDFTeX
  \RequirePackage[T1]{fontenc}
  \RequirePackage{newtxtext}   % Times-compatible text for pdfLaTeX
  \RequirePackage{newtxmath}   % Times-compatible math for pdfLaTeX
\else
  \RequirePackage{fontspec}
  \IfFontExistsTF{Times New Roman}
    {\setmainfont{Times New Roman}}
    {\setmainfont{TeX Gyre Termes}}
  \RequirePackage{unicode-math}
  \IfFontExistsTF{TeX Gyre Termes Math}
    {\setmathfont{TeX Gyre Termes Math}}
    {}
\fi

\RequirePackage{textcomp}
\RequirePackage[tiny,rm,pagestyles]{titlesec}
\RequirePackage{indentfirst}
\RequirePackage{enumitem}
\RequirePackage{ccaption}
\RequirePackage{amsmath}
\RequirePackage{graphicx}
\RequirePackage[pagewise,mathlines]{lineno}
\RequirePackage{geometry}
\RequirePackage{lastpage}
\RequirePackage[round,authoryear]{natbib}
\RequirePackage{xparse}
\RequirePackage{etoolbox}
\RequirePackage[hidelinks]{hyperref}
\RequirePackage{xurl}
\urlstyle{same}

\AtBeginDocument{%
  \ifpaper@numbered
    \linenumbers
  \fi
}
\renewcommand\linenumberfont{\normalfont\small}

% ----------------------------------------------------------------------
% Page geometry and basic formatting
% ----------------------------------------------------------------------
\geometry{
  letterpaper,
  margin=1in,
  headheight=0.3in,
  headsep=0.2in,
  footskip=0.35in
}

\setlength{\parindent}{0.5in}
\setlength{\parskip}{0pt}
\setlength{\mathindent}{0pt}
\linespread{1}
\setcounter{secnumdepth}{0}

% Running header and centered footer page number, matching the Word template.
\ExplSyntaxOn
\seq_new:N \g_paper_author_header_seq
\tl_new:N \g_paper_author_headers_tl

\cs_new_protected:Npn \paper_add_header_author:n #1
  {
    \tl_set:Nn \l_tmpa_tl {#1}
    \tl_trim_spaces:N \l_tmpa_tl
    \seq_set_split:NnV \l_tmpa_seq { ~ } \l_tmpa_tl
    \seq_gput_right:Nx \g_paper_author_header_seq
      { \seq_item:Nn \l_tmpa_seq { \seq_count:N \l_tmpa_seq } }
    \paper_update_author_headers:
  }

\cs_new_protected:Npn \paper_update_author_headers:
  {
    \int_case:nnF { \seq_count:N \g_paper_author_header_seq }
      {
        {0}
          { \tl_gclear:N \g_paper_author_headers_tl }
        {1}
          {
            \tl_gset:Nx \g_paper_author_headers_tl
              { \seq_item:Nn \g_paper_author_header_seq {1} }
          }
        {2}
          {
            \tl_gset:Nx \g_paper_author_headers_tl
              {
                \seq_item:Nn \g_paper_author_header_seq {1}
                ~ and ~
                \seq_item:Nn \g_paper_author_header_seq {2}
              }
          }
        {3}
          {
            \tl_gset:Nx \g_paper_author_headers_tl
              {
                \seq_item:Nn \g_paper_author_header_seq {1}
                , ~
                \seq_item:Nn \g_paper_author_header_seq {2}
                , ~ and ~
                \seq_item:Nn \g_paper_author_header_seq {3}
              }
          }
      }
      {
        \tl_gset:Nx \g_paper_author_headers_tl
          { \seq_item:Nn \g_paper_author_header_seq {1} ~ et~al. }
      }
  }

\NewDocumentCommand{\AuthorHeaders}{m}
  { \tl_gset:Nn \g_paper_author_headers_tl {#1} }
\NewDocumentCommand{\paperAuthorHeaders}{} 
  { \tl_use:N \g_paper_author_headers_tl }
\NewDocumentCommand{\paperAddHeaderAuthor}{m}
  { \paper_add_header_author:n {#1} }
\ExplSyntaxOff

\newpagestyle{main}{%
  \sethead{{\itshape\paperAuthorHeaders}}{}{}%
  \setfoot{}{\thepage}{}%
}
\pagestyle{main}

% ----------------------------------------------------------------------
% Headings
% ----------------------------------------------------------------------
\renewcommand*{\refname}{\uppercase{References}}
\titleformat{\section}{\bfseries}{}{0pt}{\uppercase}
\titlespacing*{\section}{0pt}{12pt}{*0}
\titleformat{\subsection}{\bfseries}{}{0pt}{}
\titlespacing*{\subsection}{0pt}{12pt}{*0}
\titleformat{\subsubsection}{\itshape}{}{0pt}{}
\titlespacing*{\subsubsection}{0pt}{12pt}{*0}

% ----------------------------------------------------------------------
% Referencing sections
% ----------------------------------------------------------------------
\newcommand{\customref}[2]{\hyperref[#2]{#1}}

% ----------------------------------------------------------------------
% Equations
% ----------------------------------------------------------------------
\g@addto@macro\normalsize{%
  \setlength\abovedisplayskip{6pt}%
  \setlength\belowdisplayskip{6pt}%
  \setlength\abovedisplayshortskip{3pt}%
  \setlength\belowdisplayshortskip{3pt}%
}

% ----------------------------------------------------------------------
% Lists
% ----------------------------------------------------------------------
\setlist[1]{labelindent=0.5in,leftmargin=*}
\setlist[2]{labelindent=0in,leftmargin=*}
\setlist{nosep}

% ----------------------------------------------------------------------
% Captions
% ----------------------------------------------------------------------
\renewcommand{\fnum@figure}{\bfseries Figure~\thefigure}
\renewcommand{\fnum@table}{\bfseries TABLE~\thetable}
\captiontitlefont{\bfseries \boldmath}
\captiondelim{ }

% ----------------------------------------------------------------------
% Citations and references
% ----------------------------------------------------------------------
% Chicago-style author-date callouts: (Smith 2020; Jones 2019).
\bibpunct{(}{)}{;}{a}{ }{, }
\setlength{\bibsep}{0pt plus 0.3ex}
\setlength{\bibhang}{0.5in}

% ----------------------------------------------------------------------
% Title-page metadata
% ----------------------------------------------------------------------
\def\@submissiondate{\today}
\newcommand{\submissiondate}[1]{\def\@submissiondate{#1}}

\def\@totalpages{\pageref{LastPage}}

\newif\ifpaper@havecorr
\paper@havecorrfalse
\newcommand*\paper@authors{}%
\def\paper@append#1{\g@addto@macro\paper@authors{#1}}%

\@ifpackageloaded{hyperref}{%
  \newcommand\paper@email[1]{\href{mailto:#1}{#1}}%
}{%
  \newcommand\paper@email[1]{\texttt{#1}}%
}

\def\paper@requireemail#1{%
  \ifblank{#1}{%
    \PackageError{trbunofficial}{Empty email in \string\author}{%
      Use \string\author*{Name}{Job title}{Affiliation}{email}[Address][ORCID]
      for the corresponding author or \string\author{...} otherwise.}%
  }{}%
}

\def\paper@mark@none{}%
\def\paper@mark@star{\textsuperscript{*}}%

% Build one author block.
%   #1 corresponding-author mark
%   #2 name
%   #3 job title
%   #4 affiliation
%   #5 email
%   #6 address or \relax
%   #7 ORCID or \relax
\def\paper@build@author#1#2#3#4#5#6#7{%
  \paper@requireemail{#5}%
  \paperAddHeaderAuthor{#2}%
  \paper@append{%
    \textbf{#2}#1\\%
    \ifblank{#3}{}{#3\\}%
    #4\\%
    \ifx\relax#6\relax\else #6\\\fi
    \ifx\relax#7\relax\else ORCID:~#7\\\fi
    Email:~\paper@email{#5}%
    \\[12pt]%
  }%
  \ifx#1\paper@mark@star
    \global\paper@havecorrtrue
  \fi
}

% Public author command:
%   \author*{Name}{Job title}{Affiliation}{email}[Address][ORCID]
\def\author{\@ifstar{\paper@author@star}{\paper@author@nostar}}
\def\paper@author@nostar#1#2#3#4{%
  \paper@optA@afterEmail{\paper@mark@none}{#1}{#2}{#3}{#4}%
}
\def\paper@author@star#1#2#3#4{%
  \paper@optA@afterEmail{\paper@mark@star}{#1}{#2}{#3}{#4}%
}

\def\paper@optA@afterEmail#1#2#3#4#5{%
  \@ifnextchar[{\paper@optAddr@afterEmail{#1}{#2}{#3}{#4}{#5}}%
               {\paper@build@author{#1}{#2}{#3}{#4}{#5}\relax\relax}%
}
\def\paper@optAddr@afterEmail#1#2#3#4#5[#6]{%
  \@ifnextchar[{\paper@optOrcid@afterAddr{#1}{#2}{#3}{#4}{#5}{#6}}%
               {\paper@build@author{#1}{#2}{#3}{#4}{#5}{#6}\relax}%
}
\def\paper@optOrcid@afterAddr#1#2#3#4#5#6[#7]{%
  \paper@build@author{#1}{#2}{#3}{#4}{#5}{#6}{#7}%
}

% ----------------------------------------------------------------------
% Title-page footnotes
% ----------------------------------------------------------------------
\newcommand*\paper@titlefootnotes{}%
\NewDocumentCommand{\titlefootnote}{m}{%
  \g@addto@macro\paper@titlefootnotes{%
    \begingroup
      \renewcommand\thefootnote{}%
      \renewcommand\@makefntext[1]{\parindent=0pt\noindent ##1}%
      \footnotetext[0]{#1}%
    \endgroup
  }%
}
\newcommand*\paper@printTitleFootnotes{%
  \paper@titlefootnotes
  \gdef\paper@titlefootnotes{}%
}

% ----------------------------------------------------------------------
% Title page
% ----------------------------------------------------------------------
\renewcommand{\maketitle}{%
  \thispagestyle{main}%
  \begin{flushleft}
    {\bfseries\@title}\\
    \hfill\break%
    \paper@authors
    \hfill\break%
    Total Number of Pages:~\@totalpages\\
    \hfill\break%
    \hfill\break%
    \textit{Submitted \@submissiondate}\\
    \ifpaper@havecorr
      \textsuperscript{*}Corresponding Author\\
    \fi
  \end{flushleft}
  \paper@printTitleFootnotes
  \newpage
}

% ----------------------------------------------------------------------
% AI disclosure helper for acknowledgments or methods, when applicable.
% ----------------------------------------------------------------------
\NewDocumentCommand{\AIDisclosure}{m}{%
  \par\noindent\textbf{AI disclosure:}~#1\par
}

% Appendices and supplemental material are not permitted for this format.
\renewcommand{\appendix}{%
  \PackageError{trbunofficial}{Appendices are not permitted for this format}{%
    Remove appendices and supplemental material from the manuscript.%
  }%
}
