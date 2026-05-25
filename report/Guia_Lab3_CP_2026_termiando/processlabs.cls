\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{processlabs}[2026/04/29]
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\LoadClass[oneside,12pt]{article}

\RequirePackage[utf8]{inputenc}
\RequirePackage[T1]{fontenc}
\RequirePackage[spanish,es-nodecimaldot]{babel}
\RequirePackage{geometry}
\RequirePackage{fancyhdr}
\RequirePackage{graphicx}
\RequirePackage{amsmath,amssymb}
\RequirePackage{xcolor}
\RequirePackage{setspace}
\RequirePackage{enumitem}
\RequirePackage{makecell}
\RequirePackage{array,tabularx,booktabs,multirow,longtable}
\RequirePackage{caption}
\RequirePackage{float}
\RequirePackage{hyperref}

\newcommand{\course}{Control de Procesos}
\newcommand{\labnumber}{3}
\newcommand{\labname}{Control LQR de un péndulo rotatorio invertido}
\newcommand{\period}{2026 -- 1}
\newcommand*{\firststudent}[1]{\def\firststudentname{#1}}
\newcommand*{\secondstudent}[1]{\def\secondstudentname{#1}}
\newcommand*{\thirdstudent}[1]{\def\thirdstudentname{#1}}
\newcommand*{\fourthstudent}[1]{\def\fourthstudentname{#1}}
\firststudent{Nombre 1}
\secondstudent{Nombre 2}
\thirdstudent{}
\fourthstudent{}

\geometry{papersize={210mm,300mm},left=30mm,right=30mm,top=30mm,bottom=30mm,heightrounded}
\pagestyle{fancy}
\fancyhf{}
\setlength{\headheight}{15pt}
\lhead{\textit{\course}}
\rfoot{-~\thepage~-}
\lfoot{\textit{Laboratorio~\labnumber:~\labname}}
\renewcommand{\headrulewidth}{0.5pt}
\renewcommand{\footrulewidth}{0.5pt}
\setstretch{1.15}
\captionsetup[figure]{labelfont={bf},name={Figura},justification=centering}

\newcommand{\makefrontpage}{%
\thispagestyle{empty}
\begin{center}
{\LARGE\textbf{Universidad de Ingeniería y Tecnología}\\[0.5cm]
\Large Departamento de Ingeniería Electrónica\\[1.6cm]}
\includegraphics[width=.48\textwidth]{images/utec_logo.png}\\[2cm]

{\LARGE\textbf{Control de Procesos}}\\[1cm]
{\LARGE\textbf{Laboratorio \labnumber}}\\[1em]
{\Large \textbf{\labname}}\\[2cm]
{\Large\textbf{Instructor:}}\\ \vspace{1em}
{\Large Sergio Morales}\\[1.5cm]
{\Large\textbf{Integrantes:}}\\ \vspace{1em}
{\Large \firststudentname}\\ \vspace{.5em}
{\Large \secondstudentname}\\[2cm]
{\Large \textbf{Lima, Perú}}\\[1.5em]
{\LARGE\textbf{\period}}
\end{center}
\newpage}

\renewcommand{\maketitle}{%
\begin{center}
\vspace*{0.1cm}
{\LARGE \textbf{Laboratorio~\labnumber}}\\[0.7em]
{\Large \textbf{\labname}}\\[1cm]
\end{center}}

\newcolumntype{Y}{>{\raggedright\arraybackslash}X}
\newcommand{\rubricsection}[1]{\multicolumn{5}{|l|}{\textbf{#1}}\\\hline}
