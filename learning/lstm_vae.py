from keras.models import Sequential
from keras.models import Model
from keras.layers import LSTM, Input, Dense, RepeatVector, TimeDistributed

input_dim = 100

inp = Input(shape=(input_dim, 1))

encoder = LSTM(100, activation='relu')(inp)

decoder_rec = RepeatVector(input_dim)(encoder)
decoder_rec = LSTM(100, activation='relu', return_sequences=True)(decoder_rec)
decoder_rec = TimeDistributed(Dense(1))(decoder_rec)

decoder_pred = RepeatVector(input_dim-1)(encoder)
decoder_pred = LSTM(100, activation='relu', return_sequences=True)(decoder_pred)
decoder_pred = TimeDistributed(Dense(1))(decoder_pred)

model = Model(inputs=inp, outputs=[decoder_rec, decoder_pred])
model.compile(optimizer='adam', loss='mse')



