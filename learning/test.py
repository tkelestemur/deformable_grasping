import torch
from lstm_autoencoder import CompositeModel, LSTMEncoder
from data import GraspDataset
from torch.utils.data import DataLoader


def test():

    device = torch.device('cuda')

    model = CompositeModel(
        input_size=5,
        hidden_size=256,
        sequence_length=375,
        num_layers=1,
        batch_first=True
    ).to(device)

    test_set = GraspDataset('test')

    data_loader_test = DataLoader(
        dataset=test_set,
        batch_size=len(test_set),
        shuffle=False,
        num_workers=4,
        drop_last=False
    )

    rec_loss, pred_loss = model.evaluate_model(
        data_loader=data_loader_test,
        checkpoint_path='./checkpoints/checkpoint.pt',
        device=device
    )

    print('Reconstruction loss: {} Prediction loss: {}'.format(rec_loss, pred_loss))


if __name__ == '__main__':
    test()
