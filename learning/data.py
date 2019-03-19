import os
import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset

PATH = os.path.dirname(os.path.realpath(__file__))


class GraspDataset(Dataset):

    def __init__(self, type=None):

        if type == 'train':
            start_idx = 0
            end_idx = 20000
        elif type == 'valid':
            start_idx = 20000
            end_idx = 25000
        elif type == 'test':
            start_idx = 25000
            end_idx = 30000
        else:
            print('Unkown type..')
            return

        print('Loading the ' + type + ' dataset...')
        data_path = PATH + '/data/states_30k.csv'
        trajectories = pd.read_csv(data_path, header=None)

        traj_length = 751
        data = []
        for i in range(start_idx, end_idx):
            traj = trajectories.iloc[i * traj_length + 1: i * traj_length + traj_length, [1, 2, 5, 6, 7]]

            traj = traj.values.tolist()
            data.append(traj)

        self.data = np.array(data)

    def __len__(self):
        return self.data.shape[0]

    def __getitem__(self, item):
        return torch.Tensor(self.data[item])


class DataRecorder:
    def __init__(self, column=None, index=None):
        self.column = column
        self.index = index
        self.df = pd.DataFrame(columns=self.column, index=self.index)

    def append(self, data_list):
        self.df.loc[self.df.shape[0]] = data_list

    def save_to_csv(self, output_path, index=False):
        self.df.to_csv(output_path, index=index)


# class ProcessData:
#
#     def __init__(self):
#         print('Loading the dataset...')
#         data_path = PATH + '/data/states_30k.csv'
#
#         self.raw_data = pd.read_csv(data_path, header=None)
#         traj_length = 751
#         num_traj = 30000
#         data = []
#         for i in range(num_traj):
#             traj = self.raw_data.iloc[i * traj_length + 1: i * traj_length + traj_length, [1, 2, 5, 6, 7]]
#
#             traj = traj.values.tolist()
#             data.append(traj)
#
#         self.data = np.array(data)
#         print('Dataset Shape: {}'.format(self.data.shape))
#
#     def generate_data(self):
#
#         train_data = self.data[0:20000]
#         valid_data = self.data[20000:25000]
#         test_data = self.data[25000:30000]




# if __name__ == '__main__':
#     process_data = ProcessData()












