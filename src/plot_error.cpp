#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>

void plot_data(const std::vector<double>& x, const std::vector<double>& y)
{
    // Create a temporary file to hold the data
    std::ofstream data_file("data.csv");
    if (!data_file.is_open()) {
        std::cerr << "Failed to create data file!" << std::endl;
        return;
    }

    // Write the data into the file
    for (size_t i = 0; i < x.size(); ++i) {
        data_file << x[i] << "," << y[i] << "\n";
    }
    data_file.close();

    // Use gnuplot to plot the data and output it as a PNG file
    std::string command = "gnuplot -e \"set datafile separator ','; set terminal png; set output 'output.png'; plot 'data.csv' using 1:2 with linespoints title 'Data'\"";
    std::cout << "Running command: " << command << std::endl;

    // Execute the gnuplot command
    int result = std::system(command.c_str());
    if (result != 0) {
        std::cerr << "Error: gnuplot command failed with exit code " << result << std::endl;
    }
}

int main() {
    std::ifstream file("/sim_ws/src/lidar/src/lidar_signal.csv");
    std::string line;
    std::vector<double> error_data;

    // Read data from CSV (assumes one column of data)
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double error;
        ss >> error;
        error_data.push_back(error);
    }

    // Assuming time or x values are sequential integers (1, 2, 3,...)
    std::vector<double> time_data(error_data.size());
    for (size_t i = 0; i < error_data.size(); ++i) {
        time_data[i] = static_cast<double>(i + 1);
    }

    // Plot the data
    plot_data(time_data, error_data);

    return 0;
}
