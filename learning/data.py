import os
import pandas as pd
import numpy as np
import torch
from torch.utils.data import Dataset

PATH = os.path.dirname(os.path.realpath(__file__))


class GraspDataset(Dataset):

    def __init__(self, type=None):
        if type == 'train':
            data_path = PATH + '/data/train_20k.csv'
            num_traj = 20000
        elif type == 'valid':
            data_path = PATH + '/data/valid_5k.csv'
            num_traj = 5000
        elif type == 'test':
            data_path = PATH + '/data/test_5k.csv'
            num_traj = 5000
        else:
            print('Unkown type..')
            return

        print('Loading the ' + type + ' dataset...')

        trajectories = pd.read_csv(data_path, header=None)

        traj_length = 751
        data = []
        for i in range(num_traj):
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


# if __name__ == '__main__':
#     print('Loading the dataset...')
#     data_path = PATH + '/data/states_30k.csv'
#     trajectories = pd.read_csv(data_path, header=None)
#
#     train_set = trajectories.iloc[0:20000*751, :]
#     valid_set = trajectories.iloc[20000*751:25000*751, :]
#     test_set = trajectories.iloc[25000*751:30000*751, :]
#
#     train_set.to_csv('./data/train_20k.csv', index=False)
#     valid_set.to_csv('./data/valid_5k.csv', index=False)
#     test_set.to_csv('./data/test_5k.csv', index=False)













