import matplotlib.pyplot as plt
import pandas as pd
plt.style.use('ggplot')


def plot_loss():
    loss = pd.read_csv('./loss/loss.csv')

    fig, axes = plt.subplots(2, 1)
    fig.suptitle('[Training set: 20k - Validation set: 5k]')
    fig.set_size_inches(12, 8)
    axes[0].plot(loss['train_reco_loss'])
    axes[0].plot(loss['vali_reco_loss'])
    axes[0].set_title('Reconstruction Loss')
    # axes[0].set_xlabel('Number of Epochs')
    axes[0].set_ylabel('Loss')
    axes[0].legend(('training', 'validation'))

    axes[1].plot(loss['train_pred_loss'])
    axes[1].plot(loss['vali_pred_loss'])
    axes[1].set_title('Prediction Loss')
    axes[1].set_xlabel('Number of Epochs')
    axes[1].set_ylabel('Loss')
    axes[1].legend(('training', 'validation'))

    # axes[2].plot(loss['train_loss'])
    # axes[2].plot(loss['vali_pred_loss']+loss['vali_pred_loss'])
    # axes[2].set_title('Total Loss')
    # axes[2].set_xlabel('Number of Epochs')
    # axes[2].set_ylabel('Loss')

    # fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    plot_loss()
