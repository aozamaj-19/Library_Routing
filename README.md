Overview

This project analyzes and optimizes the daily delivery routes of the Somerset County Library System of New Jersey (SCLSNJ) three-van courier network. The goal was to reduce total system-wide travel time while maintaining all operational constraints, including pickup schedules, inter-branch transfers, and route coverage requirements.

Using operations research principles and algorithmic modeling, I redesigned the routing structure and achieved a 17 percent reduction in total travel time, improving both efficiency and sustainability.

Features

Data cleaning and preprocessing of stop locations, time windows, and inter-branch transfer requirements

Graph modeling of the library system using weighted edges representing travel times

Route optimization using heuristics inspired by the Vehicle Routing Problem (VRP)

Load balancing across three vans to avoid bottlenecks

Scenario testing, including peak and off-peak traffic conditions

Visualized output routes and comparisons against the original routing structure

Motivation

SCLSNJ transports thousands of materials across its branches daily. Before optimization, the courier system resembled a puzzle with extra steps and unnecessary detours. This project attempts to “zoom out” and reveal a cleaner, faster pattern.

As someone who enjoys solving complex systems and finding the quickest path between points—whether on a hiking trail or in a network graph—this project was a natural fit.

Methodology

Data Collection

Branch locations and latitude/longitude

Existing van routes and schedules

Historical trip durations

Model Construction

Built a weighted graph using travel-time estimates

Encoded constraints (branch deadlines, required pickups, van capacities)

Optimization Approach

Developed routing heuristics based on VRP

Used iterative improvement (2-opt, swap, and cluster-first route-second strategies)

Evaluated results with objective metrics: total time, distance, load distribution

Validation

Ran simulations under varying traffic conditions

Cross-checked that all operational constraints remained satisfied

Results

17 percent reduction in total travel time

Better balanced van workloads

More predictable and consistent routing

Reduced fuel usage, contributing to sustainability goals
