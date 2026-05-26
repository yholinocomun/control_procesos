\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{processlabs}[2026/05/14 Process Control Laboratory guides]
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\LoadClass[12pt,oneside]{article}

\RequirePackage[utf8]{inputenc}
\RequirePackage[T1]{fontenc}
\RequirePackage[spanish,es-nodecimaldot]{babel}
\RequirePackage{geometry}
\RequirePackage{fancyhdr}
\RequirePackage{graphicx}
\RequirePackage{array}
\RequirePackage{tabularx}
\RequirePackage{enumitem}
\RequirePackage{setspace}
\RequirePackage{titlesec}
\RequirePackage[hidelinks]{hyperref}
\RequirePackage{caption}
\RequirePackage{float}

\geometry{
  papersize={210mm,300mm},
  left=30mm,
  right=30mm,
  top=30mm,
  bottom=30mm,
  heightrounded
}

\setstretch{1.15}
\setlength{\parindent}{0pt}
\setlength{\parskip}{0.3em}
\captionsetup[figure]{labelfont=bf,name=Figura}

\newcommand{\course}{Laboratorio de Control de Procesos}
\newcommand{\labnumber}{4}
\newcommand{\labname}{Control de ratio de caudal y control en cascada de nivel a caudal}
\newcommand{\doctype}{Prelaboratorio}
\newcommand{\period}{2026 -- 1}
\newcommand{\instructor}{Sergio Morales}
\newcommand{\studentname}{}
\newcommand*{\student}[1]{\gdef\studentname{#1}}

\pagestyle{fancy}
\fancyhf{}
\setlength{\headheight}{15pt}
\lhead{\textit{\doctype~\labnumber{} -- \labname}}
\rfoot{\thepage}
\lfoot{\textit{Universidad de Ingeniería y Tecnología - UTEC}}
\renewcommand{\headrulewidth}{0.4pt}
\renewcommand{\footrulewidth}{0.4pt}

\titleformat{\section}{\normalfont\large\bfseries}{\thesection.}{0.5em}{}
\titleformat{\subsection}{\normalfont\normalsize\bfseries}{\thesubsection.}{0.5em}{}

\newcommand{\makefrontpage}{%
  \thispagestyle{empty}
  \begin{center}
    {\LARGE\textbf{Universidad de Ingeniería y Tecnología}\\[0.55cm]}
    {\Large Departamento de Ingeniería Electrónica\\[1.35cm]}
    \includegraphics[width=0.36\textwidth]{images/utec_logo.png}\\[1.65cm]
    {\LARGE\textbf{\course}\\[1.05cm]}
    {\Large\textbf{Instructor:}\\[0.15cm]}
    {\Large \instructor\\[1.0cm]}
    {\LARGE\textbf{\doctype~\labnumber}\\[0.2cm]}
    {\Large\textbf{\labname}\\[1.15cm]}
    {\large\textbf{Nombre y Apellido:}\\[0.3cm]}
    {\large \studentname\\[1.9cm]}
    {\Large \textbf{Lima, Perú}\\[0.7cm]}
    {\LARGE \textbf{\period}}
  \end{center}
  \newpage
}

\renewcommand{\maketitle}{%
\begin{center}
\vspace*{0.1cm}
{\LARGE \textbf{Prelaboratorio~\labnumber}}\\[0.7em]
{\Large \textbf{\labname}}\\[1cm]
\end{center}}
