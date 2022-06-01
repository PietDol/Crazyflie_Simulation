from Crazyflie_Simulation.solid.log import Analyse

analyse = Analyse(filename="log.csv", directory="../Crazyflie_Simulation/solid/Logging/")

# id = a unique number to identify dataset with the calculated differences: like plt.figure('number')
# run1 and run2 = the numbers in at the top of the csv-file given during filling the file
# So it is possible the fill the cell with lots of runs and perform different calculations.
analyse.calculate_differences(id=1, run1=1, run2=2)
analyse.calculate_differences(id=2, run1=2, run2=1)

# id = plot the error of a certain variable against time from the dataset with the same id as above
# mode = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
# analyse.plot_graph_difference(id=1, mode="x")
# analyse.plot_graph_difference(id=2, mode="x")


analyse.plot_graph_comparison(mode="x", runs=[1,2])
# analyse.plot_graph_comparison(mode="y", runs=[1,2])
analyse.plot_graph_comparison(mode="z", runs=[1,2])
# analyse.plot_graph_comparison(mode="roll", runs=[1,2])
analyse.plot_graph_comparison(mode="pitch", runs=[1,2])
# analyse.plot_graph_comparison(mode="yaw", runs=[1,2])
