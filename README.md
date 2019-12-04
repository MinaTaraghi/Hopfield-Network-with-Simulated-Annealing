# Hopfield Network with Simulated Annealing
 Hopfield Neuran Network with Simulated Annealing Option

A hopfield network for learning some binary images in folders 1, 2, and 3

here are a number of parameters and options that can be decided on by the user through the GUI:
 - Dataset to train on (1, 2, or 3)
 - Network type (Binary, Bipolar)
 - Percentage of the noise added to a selected image for testing the network
 - Using simulated annealing for optimization or not
 - The parameters for simulated annealing (T0, Epsilon, Alpha)

After choosing the dataset and hitting the Train button, number of patterns stored and the images stored in network are displayed
For testing the network, you can use the load image button and after adding any amount of noise, hit the Run Button.
Network will try to find the nearest match to the image. You can choose to use SA or not.
The Mean Square Error for different iterations (until convergence), minimum MSE, and number of iterations are displayed afterwards.
The match found by network is also displayed on the right.
