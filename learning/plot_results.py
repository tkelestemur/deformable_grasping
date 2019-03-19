import matplotlib.pyplot as plt
import pandas as pd
plt.style.use('ggplot')


def plot_loss():
    loss = pd.read_csv('./data/loss/loss.csv')

    fig, ax_1 = plt.subplots(1, 1)
    fig.set_size_inches(10, 5)
    ax_1.plot(loss['train_pred_loss'])
    ax_1.set_title('Total Loss for 30k')
    ax_1.set_xlabel('Number of Epochs')
    ax_1.set_ylabel('Loss')
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    plot_loss()
