Written in python 3 and developed in Ubunutu 18.04 LTS

Run:
execute python3 run.py

run.py displays results to questions: 2,3,6,8,and 9

The file run.py will first display the two plots for question 8. You have the
option to specify how long to run the full trajectory. It took 40s on my
computer to run the trajectory. After the plots for part B run the plots for
Part A will run. You must close a plot for the next one to appear. 

The plot to question 2 will appear then after it is closed
the plot to question 3 will appear. Finally after closure of the last plot
the range and bearing results to 6 will print to the terminal. The error in
the global position of landmarks will be non zero if noise is add.

Dependencies:
numpy
matplotlib
scipy

Notes:
Change the data file input paths at the top of params.py to
specify a different file path.

Specifiy to dd noise to the sensor and motion models on line 155 run.py
