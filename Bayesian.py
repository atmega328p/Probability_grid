import math

class Bayesian:
    """
    Calculate probabilities for occupancy grid based on prior knowledge and sensor readings.
    """

    def __init__(self, prob_grid):
        self.prob_grid = prob_grid
        self.maximum_range = 30
        self.beta = 0.5 #Half lobe angle
        self.accuracy = 0.01
        self.p_max = 0.98
        self.robot_row = 0
        self.robot_col = 0

    def bayes_handler(self, bresenham_line, robot_row, robot_col):
        """
        Divides data, handles flow of calulations
        :param brasenham_line (list of tuples): A single brasenhamline of tuples
        :param robot_row:
        :param robot_col:
        :return:
        """
        self.robot_row = robot_row
        self.robot_col = robot_col

        # Pick last element in (sensor reading cell)
        sensor_cell = bresenham_line[len(bresenham_line) - 1]

        # Iterate of cells (tuple) to calculate bayes probability
        for i in range(0, len(bresenham_line)):
            # Calculate probability for a cell to be occupied
            p_occupied = self.bayes_rule(sensor_cell, bresenham_line[i])

            # Update probability for a cell to be occupied
            prob_grid_x = bresenham_line[i][0]
            prob_grid_y = bresenham_line[i][1]
            self.prob_grid[prob_grid_x][prob_grid_y] = p_occupied

    def bayes_rule(self, sensor_cell, grid_cell):
        """
        Input:
        @ sensor_cell (tuple): One cell (x,y) for current laser reading, (last element in Brasenham list)
        @ grid_cell (tuple): Probability grid cell in Brasenham line (x,y)

        P(Occupied | s) = P(s|Occupied)*P(Occupied)/(P(s|Occupied)*P(Occupied) + P(s|!Occupied)*P(!Occupied))
        P(Occupied, s) = P(s,Occupied)*P(Occupied)/(P(s,Occupied)*P(Occupied) + P(s,!Occupied)*P(!Occupied))

        Output:
        P(Occupied|s)
        """
        print_it = 0

        # Decide what region the grid cell belongs to
        region = self.decide_region(grid_cell, sensor_cell)

        dist_robot_cell_to_grid_cell = math.sqrt((grid_cell[0] - self.robot_row)**2 + (grid_cell[1] - self.robot_col)**2)

        if region == 1:
            # regionI: A region at the cell/region around the coordinates of the laser reading
            if print_it == 1:
                print("regionI", end=' ')
                print("{:.4f}".format(self.regionI_occupied(dist_robot_cell_to_grid_cell, 0)))

            PsH = self.regionI_occupied(dist_robot_cell_to_grid_cell, 0)
            PH =  self.probability(grid_cell)

            return PsH*PH/(PsH*PH + (1-PsH)*(1-PH))

        elif region == 2:
            # regionII: A region from robot along LIDAR angle until the laser reading
            if print_it == 1:
                print("regionII", end=' ')
                print("{:.4f}".format(self.regionII_occupied(dist_robot_cell_to_grid_cell, 0)))

            PsH = self.regionII_occupied(dist_robot_cell_to_grid_cell, 0)
            PH =  self.probability(grid_cell)

            return PsH*PH/(PsH*PH + (1-PsH)*(1-PH))

        elif region == 3:
            pass

        else:
            pass # Do not update

    def regionI_occupied(self, r, alpha):
        """
        :param r:
        :param alpha:
        :return:
        """
        return self.regionII_empty(r, alpha)*self.p_max

    def regionI_empty(self, r, alpha):
        """

        :param r:
        :param alpha:
        :return:
        """
        return 1 - self.regionI_occupied(r, alpha)

    def regionII_occupied(self, r, alpha):
        """

        :param r:
        :param alpha:
        :return:
        """
        return 1-self.regionII_empty(r, alpha)

    def regionII_empty(self, r, alpha):
        """

        :param r:
        :param alpha:
        :return:
        """
        a = (self.maximum_range - r)/self.maximum_range
        b = (self.beta - abs(alpha))/self.beta
        return (a + b)/2

    def probability(self, cell):
        """
        Gets the value of the current probability for a grid cell
        :param cell (tuple): A position in the probability grid (x,y)
        :return: Probability of an occupied cell
        """
        return self.prob_grid[cell[0]][cell[1]]

    def decide_region(self, grid_cell, sensor_cell):
        """
        :param grid_cell: The coordinate along y for the current grid cell
        :param sensor_cell:
        :return:
        """
        dist_robot_to_sensor_cell = math.sqrt( (sensor_cell[0]-self.robot_row) ** 2 + (sensor_cell[1]-self.robot_col)** 2)
        dist_grid_cell_to_sensor_cell = math.sqrt((sensor_cell[0] - grid_cell[0]) ** 2 + (sensor_cell[1] - grid_cell[1]) ** 2)
        dist_robot_cell_to_grid_cell = math.sqrt(
            (grid_cell[0] - self.robot_row) ** 2 + (grid_cell[1] - self.robot_col) ** 2)

        if dist_robot_cell_to_grid_cell > ( dist_robot_to_sensor_cell + self.accuracy):
            return 3  # Beyond sensor reading
        elif dist_grid_cell_to_sensor_cell <= self.accuracy:
            return 1
        elif dist_robot_to_sensor_cell > dist_robot_cell_to_grid_cell:
            return 2
        else:
            return 0 #Error or beyond sensor reading


    def print_prob_grid(self):

        for row in range(0, len(self.prob_grid)):
            for col in range(0, len(self.prob_grid)):
                print("{:.1f}".format((self.prob_grid[row][col])), end = ' ')

            print()

