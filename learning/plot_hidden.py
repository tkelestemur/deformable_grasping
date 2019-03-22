import torch
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from lstm_autoencoder import CompositeModel, LSTMEncoder
from data import GraspDataset
from sklearn.decomposition import PCA
from sklearn.manifold import TSNE
plt.style.use('ggplot')

device = torch.device('cuda')

model = CompositeModel(
    input_size=5,
    hidden_size=256,
    sequence_length=375,
    num_layers=1,
    batch_first=True
).to(device)

model.load_state_dict(torch.load('./checkpoints/checkpoint_032019.pt'), strict=True)
test_set = GraspDataset('test')

data_loader_test = DataLoader(
    dataset=test_set,
    batch_size=100,
    shuffle=False,
    num_workers=4,
    drop_last=False
)


encoded_output_total = []

for i, data in enumerate(data_loader_test):
    print(i)
    input_data = data[:, :375, :]
    input_data = np.flip(input_data.numpy(), axis=1)
    input_data = torch.from_numpy(input_data.copy())
    input_data = input_data.to(device)

    encoded_output, _, _ = model.forward(input_data)

    encoded_output_total.append(encoded_output.detach().cpu().numpy().squeeze())

encoded_output_total = np.array(encoded_output_total)



# encoded_output_total = []
#
# for i in range(len(test_set)):
#     data = test_set[i].reshape(1, 750, 5)
#     input_data = data[:, :375, :]
#     input_data = np.flip(input_data.numpy(), axis=1)
#     input_data = torch.from_numpy(input_data.copy())
#     input_data = input_data.to(device)
#
#     encoded_output, _, _ = model.forward(input_data)
#     if i+1 % 100 == 0:
#         print((i+1)*100)
#     encoded_output_total.append(encoded_output.detach().cpu().numpy().squeeze())
#
# encoded_output_total = np.array(encoded_output_total)


def pca():
    pca = PCA(n_components=2)

    # pca_data = encoded_output.detach().cpu().numpy().squeeze()
    print(encoded_output_total.shape)
    transformed_data = pca.fit_transform(encoded_output_total.reshape(5000, 256))

    plt.figure(figsize=(8, 8))
    plt.scatter(transformed_data[:, 0], transformed_data[:, 1], c='b')

    plt.show()


def tsne():
    tsne = TSNE(n_components=2)

    # tsne_data = encoded_output.detach().cpu().numpy().squeeze()
    transformed_data = tsne.fit_transform(encoded_output_total.reshape(5000, 256))

    plt.figure(figsize=(8, 8))
    plt.title('TSNE Clustering - Test Set:5k')
    plt.scatter(transformed_data[:, 0], transformed_data[:, 1], c='r')

    plt.show()


if __name__ == '__main__':
    # pca()
    tsne()