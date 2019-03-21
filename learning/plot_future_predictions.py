import torch
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from lstm_autoencoder import CompositeModel
from data import GraspDataset
plt.style.use('ggplot')

device = torch.device('cuda')

model = CompositeModel(
    input_size=5,
    hidden_size=256,
    sequence_length=375,
    num_layers=1,
    batch_first=True
).to(device)

model.load_state_dict(torch.load('./checkpoints/checkpoint.pt'), strict=True)
test_set = GraspDataset('train')
data = test_set[7:8]

input_data = data[:, :375, :]
future_data = data[:, 375:, :]

input_data = np.flip(input_data.numpy(), axis=1)
input_data = torch.from_numpy(input_data.copy())
input_data = input_data.to(device)
future_data = future_data.to(device)

encoded_output, decoded_output, forward_output = model.forward(input_data)


def plot_future():
    future_real = future_data.detach().cpu().numpy().squeeze()
    future_pred = forward_output.detach().cpu().numpy().squeeze()

    input_real = data[:, :375, :].detach().cpu().numpy().squeeze()
    input_reco = decoded_output.detach().cpu().numpy().squeeze()
    input_reco = np.flip(input_reco, axis=0)

    fig, axes = plt.subplots(2, 1)
    fig.suptitle('[Test: 5k]')
    fig.set_size_inches(12, 8)
    axes[0].plot(np.linspace(0, 374, 375), input_real[:, 0])
    axes[0].plot(np.linspace(0, 374, 375), input_reco[:, 0])
    axes[0].set_title('Reconstruction')
    # axes[0].set_xlabel('Number of Epochs')
    axes[0].set_ylabel('Finger 1 Angle')
    axes[0].legend(('ground_truth', 'prediction'))

    axes[1].plot(np.linspace(375, 749, 375), future_real[:, 0])
    axes[1].plot( np.linspace(375, 749, 375), future_pred[:, 0])
    axes[1].set_title('Prediction')
    axes[1].set_xlabel('Timesteps')
    axes[1].set_ylabel('Finger 1 Angle')
    axes[1].legend(('ground_truth', 'prediction'))
    plt.show()


def plot_future_reco():


if __name__ == '__main__':
    plot_future()
    plot_future_reco()