# pyadrc
Active Disturbance Rejection Control for Python

A simple and easy to use Active Disturbance Rejection Control algorithm in Python.

I needed this algorithm for my Master's degree at the Technical University of Berlin and seeing how awesome it worked, I wanted to develop this Python package further for others to use, contribute and have fun with it! There is a vast amount of really good literature on this topic, from different tracking differentiators to nonlinear state feedback functions. The plan is to implement as much of these as possible without scope-creep.

## Introduction

Active Disturbance Rejection Control (ADRC) is a model-free control paradigm, proposed by Jingqing Han in his work "From PID to Active Disturbance Rejection Control" [[1]](#1). However, for my Master's thesis, I implemented *linear active disturbance rejection control* in discrete form, as described by Gernot Herbst in "A Simulative Study on Active Disturbance Rejection Control (ADRC) as a Control Tool for Practitioners" [[2]](#2). 

## References
<a id="1">[1]</a> 
Jingqing Han (March 2009). 
From PID to Active Disturbance Rejection Control
doi:10.1109/TIE.2008.2011621

<a id="2">[2]</a> 
Gernot Herbst (15 August 2013). 
A Simulative Study on Active Disturbance Rejection Control (ADRC) as a Control Tool for Practitioners
doi:10.3390/electronics2030246
