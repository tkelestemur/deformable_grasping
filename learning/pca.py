import torch
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from lstm_autoencoder import CompositeModel, LSTMEncoder
from data import GraspDataset
from sklearn.decomposition import PCA
from sklearn.manifold import TSNE


def pca():
    pca = PCA(n_components=2)
    tsne = TSNE(n_components=2)
    device = torch.device('cuda')

    model = CompositeModel(
        input_size=5,
        hidden_size=256,
        sequence_length=375,
        num_layers=1,
        batch_first=True
    ).to(device)
    model.load_state_dict(torch.load('./checkpoints/checkpoint_stable.pt'), strict=True)
    test_set = GraspDataset('test')
    data_loader_test = DataLoader(
        dataset=test_set,
        batch_size=len(test_set),
        shuffle=False,
        num_workers=4,
        drop_last=False
    )

    for data in data_loader_test:
        input_data = data[:, :375, :]
        future_data = data[:, 375:, :]
        print(input_data.shape)
        input_data = np.flip(input_data.numpy(), axis=1)
        input_data = torch.from_numpy(input_data.copy())
        input_data = input_data.to(device)
        # future_data = future_data.to(device)
        encoded_output, decoded_output, forward_output = model.forward(input_data)
        print(encoded_output.shape)
        print(encoded_output)
        break

    pca_data = encoded_output.detach().cpu().numpy().squeeze()
    print(pca_data.shape)
    transformed_data = tsne.fit_transform(pca_data)

    plt.figure(figsize=(8, 8))
    plt.scatter(transformed_data[:, 0], transformed_data[:, 1], c='b')

if __name__ == '__main__':
    pca()
