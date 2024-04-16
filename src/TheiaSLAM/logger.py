

class DataLog: 
    def __init__(self, file_path=None):
       self.file_path = file_path
       self.log_data = []

    def log_initial_landmark_positions_and_controls(self, landmark_positions, dx, dtheta):
        data = "Initial Landmark Positions:\n"
        for i, landmark in enumerate(landmark_positions):
            data += f"Landmark {i}: x={landmark[0]}, y={landmark[1]}\n"

        data += f"Control Inputs: dx={dx}, dtheta={dtheta}"
        self.log_data.append(data)
        print(data)

    def save_to_file(self):
        file_path = self.file_path or file_path
        if self.file_path:
            with open(file_path, 'w') as f:
                for data in self.log_data:
                    f.write(data + '\n')
            print (f"Data saved to {file_path}")
        else:
            print("No file path provided")
