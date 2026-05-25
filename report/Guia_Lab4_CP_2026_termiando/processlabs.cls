\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{processlabs}[2026/05/21]
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
\newcommand{\labnumber}{4}
\newcommand{\labname}{Control de ratio de caudal y control en cascada de nivel a caudal}
\newcommand{\shortlabname}{Control de ratio y cascada nivel-caudal}
\newcommand{\period}{2026 -- 1}
\newcommand*{\firststudent}[1]{\def\firststudentname{#1}}
\newcommand*{\secondstudent}[1]{\def\secondstudentname{#1}}
\newcommand*{\thirdstudent}[1]{\def\thirdstudentname{#1}}
\newcommand*{\fourthstudent}[1]{\def\fourthstudentname{#1}}
\firststudent{Nombre 1}
\secondstudent{Nombre 2}
\thirdstudent{Nombre 3}
\fourthstudent{Nombre 4}

\geometry{papersize={210mm,300mm},left=30mm,right=30mm,top=30mm,bottom=30mm,heightrounded}
\pagestyle{fancy}
\fancyhf{}
\setlength{\headheight}{15pt}
\lhead{\textit{\course}}
\rfoot{-~\thepage~-}
\lfoot{\footnotesize\textit{Laboratorio~\labnumber:~\shortlabname}}
\renewcommand{\headrulewidth}{0.5pt}
\renewcommand{\footrulewidth}{0.5pt}
\setstretch{1.15}
\captionsetup[figure]{labelfont={bf},name={Figura},justification=centering}
\sloppy

\newcommand{\makefrontpage}{%
\thispagestyle{empty}
\begin{center}
\vspace*{-0.8cm}
{\LARGE\textbf{Universidad de Ingeniería y Tecnología}\\[0.35cm]
\Large Departamento de Ingeniería Electrónica\\[1.15cm]}
\includegraphics[width=.43\textwidth]{images/utec_logo.png}\\[1.5cm]
{\LARGE\textbf{Laboratorio de Control de Procesos}}\\[1cm]
{\LARGE\textbf{Laboratorio \labnumber}}\\[0.35cm]
{\Large \textbf{\labname}}\\[1.5cm]
{\Large\textbf{Instructor:}}\\[0.15cm]
{\Large Sergio Morales}\\[1cm]
{\Large\textbf{Integrantes:}}\\[0.25cm]
{\Large \firststudentname}\\[0.15cm]
{\Large \secondstudentname}\\[0.15cm]
{\Large \thirdstudentname}\\[0.15cm]
{\Large \fourthstudentname}\\[1.5cm]
{\Large \textbf{Lima, Perú}}\\[0.5cm]
{\LARGE\textbf{\period}}
\end{center}
\newpage}

\renewcommand{\maketitle}{%
\begin{center}
\vspace*{0.8cm}
{\huge \textbf{Laboratorio~\labnumber}}\\[0.7em]
{\LARGE \textbf{\labname}}\\[1cm]
\end{center}}

\newcolumntype{Y}{>{\raggedright\arraybackslash}X}
\newcommand{\rubricsection}[1]{\multicolumn{5}{|l|}{\textbf{#1}}\\\hline}
