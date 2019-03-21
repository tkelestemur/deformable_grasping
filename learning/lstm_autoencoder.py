import torch
from torch import nn
from torch import optim
import numpy as np
import datetime
from data import DataRecorder


class LSTMEncoder(nn.Module):

    def __init__(self, input_size, hidden_size, num_layers, batch_first=True):
        super(LSTMEncoder, self).__init__()

        self.input_size_ = input_size
        self.hidden_size_ = hidden_size
        self.num_layers_ = num_layers

        self.encoder_ = nn.LSTM(
            input_size=input_size,  # same as size of feature vectors
            hidden_size=hidden_size,  # size of both hidden and cell vectors
            num_layers=num_layers,
            batch_first=batch_first
        )

    def forward(self, input):
        # h_0 and c_0 are initialized to zeros by default
        encoded_output, hidden = self.encoder_(input)
        # return the last hidden vector as representation
        return encoded_output[:, [-1], :], hidden


class LSTMDecoder(nn.Module):

    def __init__(self, input_size, hidden_size, output_size, sequence_length, num_layers, batch_first=True):
        super(LSTMDecoder, self).__init__()

        self.input_size_ = input_size
        self.hidden_size_ = hidden_size
        self.num_layers_ = num_layers
        self.sequence_length_ = sequence_length

        self.decoder_ = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=batch_first
        )

        self.fc_layer_ = nn.Linear(
            in_features=hidden_size,
            out_features=output_size
        )

    def forward(self, input, hidden):
        output = []
        # use the output of last sequence of encoder as first input to decoder
        # decoded_output = input
        for idx in range(self.sequence_length_):
            # initialize the hidden and cell states from encoders
            # the output is conditional on previous output
            decoded_output, hidden = self.decoder_(input, hidden)
            decoded_output = self.fc_layer_(decoded_output)
            output += [decoded_output]

        # reshape the output tensor
        decoded_output = torch.stack(output, 1).squeeze(2)

        return decoded_output, hidden


class CompositeModel(nn.Module):

    def __init__(self, input_size, hidden_size, sequence_length, num_layers, batch_first=True, loss_path=None):

        super(CompositeModel, self).__init__()

        self.input_size = input_size
        self.hidden_size = hidden_size
        self.sequence_length = sequence_length
        self.num_layers = num_layers
        self.batch_firs = batch_first
        self.loss_path = loss_path

        # Initialize an LSTM for encoder
        self.encoder_ = LSTMEncoder(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=batch_first
        )

        # Initialize an LSTM for decoder
        self.decoder_ = LSTMDecoder(
            input_size=hidden_size,
            hidden_size=hidden_size,
            output_size=input_size,
            num_layers=num_layers,
            batch_first=batch_first,
            sequence_length=sequence_length
        )

        # Initialize an LSTM for future predictions
        self.future_predictor_ = LSTMDecoder(
            input_size=hidden_size,
            hidden_size=hidden_size,
            output_size=input_size,
            num_layers=num_layers,
            batch_first=batch_first,
            sequence_length=sequence_length
        )

    def forward(self, input):
        encoded_output, hidden = self.encoder_(input)
        decoded_output, hidden = self.decoder_(encoded_output, hidden)
        forward_output, hidden = self.future_predictor_(encoded_output, hidden)
        return encoded_output, decoded_output, forward_output

    def train_model(self, num_epochs, data_loader_train, data_loader_valid, checkpoint_path,
                    lr=1e-3, weight_decay=0, sparsity_weight=0, device=torch.device('cpu')):

        loss_recorder_column = ['epoch_id', 'train_reco_loss', 'train_pred_loss', 
                                'train_loss', 'vali_reco_loss', 'vali_pred_loss']
        loss_recorder = DataRecorder(column=loss_recorder_column)

        optimizer = optim.Adam(params=self.parameters(), lr=lr, weight_decay=weight_decay)

        eval_model = CompositeModel(
            input_size=self.input_size,
            hidden_size=self.hidden_size,
            sequence_length=self.sequence_length,
            num_layers=1,
            batch_first=True
        ).to(device)

        start_time = datetime.datetime.now()

        print('Starting the training...')
        for epoch_i in range(num_epochs):
            print('Training epoch # {}'.format(epoch_i))
            train_reco_loss = None
            train_pred_loss = None
            train_loss = None
            vali_reco_loss = None
            vali_pred_loss = None

            for i_batch, curr_batch in enumerate(data_loader_train):
                # reverse the order so that the first reconstruction is compared with the last frame
                input_data = curr_batch[:, :self.sequence_length, :]
                future_data = curr_batch[:, self.sequence_length:, :]

                input_data = np.flip(input_data.numpy(), axis=1)
                input_data = torch.from_numpy(input_data.copy())
                input_data = input_data.to(device)
                future_data = future_data.to(device)

                encoded_output, decoded_output, forward_output = self.forward(input_data)

                reconstruction_criteria = nn.MSELoss()
                reconstruction_loss = reconstruction_criteria(
                    input=decoded_output,
                    target=input_data
                )
                prediction_criteria = nn.MSELoss()
                prediction_loss = prediction_criteria(
                    input=forward_output,
                    target=future_data
                )

                sparsity_loss = torch.mean(torch.pow(torch.norm(encoded_output, p=2, dim=-1), 2))

                loss = reconstruction_loss + prediction_loss + (sparsity_weight * sparsity_loss)

                train_reco_loss = reconstruction_loss.item()
                train_pred_loss = prediction_loss.item()
                train_loss = loss.item()

                self.zero_grad()
                loss.backward()
                optimizer.step()

            # save epoch checkpoint
            with open(checkpoint_path, 'wb') as f:
                torch.save(self.state_dict(), f)

            # evaluate the model at every eval_freq
            if epoch_i % 1 == 0:
                vali_reco_loss, vali_pred_loss = eval_model.evaluate_model(
                    data_loader=data_loader_valid,
                    checkpoint_path=checkpoint_path,
                    device=device)

            epoch_recorder_row = [int(epoch_i), train_reco_loss, train_pred_loss, train_loss,
                                  vali_reco_loss, vali_pred_loss]

            loss_recorder.append(epoch_recorder_row)
            loss_recorder.save_to_csv(self.loss_path, index=False)

            epochs_counter = epoch_i + 1

            if epochs_counter % 20 == 0 or epochs_counter == 1:
                current_time = datetime.datetime.now()
                left_time = (current_time - start_time).seconds / 60 / epochs_counter * (num_epochs - epochs_counter)

                if left_time > 100:
                    print("Remaining Time: {} hour(s).".format(int(left_time / 60)))
                else:
                    print("Remaining Time: {} min(s) left.".format(left_time))

    def evaluate_model(self, data_loader, checkpoint_path, device=torch.device("cpu")):

        self.load_state_dict(torch.load(checkpoint_path), strict=True)
        input_data = None
        future_data = None
        forward_output = None
        reconstruction_losses = []
        prediction_losses = []

        print('Evaluating...')

        for i_batch, curr_batch in enumerate(data_loader):
            # reverse the order so that the first reconstruction is compared with the last frame
            input_data = curr_batch[:, :self.sequence_length, :]
            future_data = curr_batch[:, self.sequence_length:, :]

            input_data = np.flip(input_data.numpy(), axis=1)
            input_data = torch.from_numpy(input_data.copy())
            input_data = input_data.to(device)
            future_data = future_data.to(device)

            encoded_output, decoded_output, forward_output = self.forward(input_data)

            reconstruction_criteria = nn.MSELoss()
            reconstruction_loss = reconstruction_criteria(
                input=decoded_output,
                target=input_data
            )

            prediction_criteria = nn.MSELoss()
            prediction_loss = prediction_criteria(
                input=forward_output,
                target=future_data
            )

            reconstruction_losses.append(reconstruction_loss.item())
            prediction_losses.append(prediction_loss.item())

        return np.mean(reconstruction_losses), np.mean(prediction_losses)
