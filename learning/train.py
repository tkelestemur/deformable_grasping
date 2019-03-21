import torch
from lstm_autoencoder import CompositeModel
from data import GraspDataset
from torch.utils.data import DataLoader


def train():

    device = torch.device('cuda')

    model = CompositeModel(
        input_size=5,
        hidden_size=256,
        sequence_length=375,
        num_layers=1,
        batch_first=True,
        loss_path='./loss/loss_032019.csv'
    ).to(device)

    train_set = GraspDataset('train')
    valid_set = GraspDataset('valid')

    data_loader_train = DataLoader(
        dataset=train_set,
        batch_size=128,
        shuffle=False,
        num_workers=4,
        drop_last=True
    )

    data_loader_valid = DataLoader(
        dataset=valid_set,
        batch_size=250,
        shuffle=False,
        num_workers=2,
        drop_last=False
    )

    model.train_model(
        num_epochs=1500,
        data_loader_train=data_loader_train,
        data_loader_valid=data_loader_valid,
        checkpoint_path='./checkpoints/checkpoint_032019.pt',
        lr=1e-3,
        weight_decay=0.000001,
        sparsity_weight=0.01,
        device=device
    )


if __name__ == '__main__':
    train()
