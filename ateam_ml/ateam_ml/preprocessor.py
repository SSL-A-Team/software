import os
import shutil
import glob
import pandas as pd
import seaborn as sn
import matplotlib.pyplot as plt
import argparse
import numpy as np

from IPython.display import display

from sklearn.decomposition import PCA
from sklearn.manifold import TSNE

class Preprocessor():
    def __init__(self):
        parser = argparse.ArgumentParser(
            prog='ProgramName',
            description='What the program does',
            epilog='Text at the bottom of help')

        parser.add_argument('csv_dir')
        parser.add_argument('output_csv_filepath')

        args = parser.parse_args()

        # I dont know why I didnt just do a glob of the *.csvs on path...

        mainframe = pd.DataFrame()
        topic_csvs = glob.glob(args.csv_dir + "/*.csv")
        # excluded_topic_list = ['client_count', 'vision_messages', 'uiclient_count', 'rosout', 'vision_messages', 'ui_connected_clients.csv', 'parameter_events']
        # topic_list = [topic for topic in topics_table if topic not in excluded_topic_list]
        for csv in topic_csvs:
            topic = os.path.splitext(os.path.basename(csv))[0]
            print(topic)
            # dataframe = pd.read_csv(csv)
            dataframe = pd.DataFrame()

            for chunk in pd.read_csv(csv, chunksize=100000, low_memory=False):
                dataframe = pd.concat([dataframe, chunk])
            # dataframe = dataframe.dropna(axis='index', how='any')
            dataframe['timestamp'] = pd.to_datetime(dataframe['timestamp'])
            dataframe = dataframe[dataframe['timestamp'] > np.datetime64(1*(10**9), 'ns')]
            dataframe = dataframe.set_index('timestamp')
            dataframe = dataframe.add_prefix(f'{topic}.')

            mainframe = pd.concat([mainframe, dataframe], axis=1, join='outer')

        print("final mainframe:\n ", mainframe)
        # mainframe.to_csv(args.output_csv_filepath)

        # gives a good idea of what you can reasonably interpolate
        times = pd.DataFrame({"times": mainframe.index.values.astype(np.ulonglong)})
        print("range: ", (times.max() - times.min()) / 10**9)
        diffs_in_seconds = (times['times'].diff() / 10**9)
        print(diffs_in_seconds.describe())


        upsampled = mainframe.resample(pd.Timedelta('1milli')).mean()
        # upsampled.to_csv(args.output_csv_filepath)


        # interpolated_frame = upsampled.interpolate(method='linear')
        # interpolated_frame = upsampled.interpolate(method='barycentric') # this one takes a small enternity

        interpolated_frame = upsampled.interpolate(method='akima')
        print("interpolated_frame:\n ", interpolated_frame)

        interpolated_frame.to_csv(args.output_csv_filepath)


        # comparison of pre interpolation to after
        # wipe = lambda df, topic: df.dropna(axis='index', how='any', subset=topic)[topic].values
        # plt.plot(wipe(interpolated_frame, "ball.pose.position.x"), wipe(interpolated_frame,"ball.pose.position.y"), 'bo', ms=1)
        # plt.plot(wipe(mainframe, "ball.pose.position.x"), wipe(mainframe,"ball.pose.position.y"), 'go', ms=1)
        # plt.show()



        # tsne = TSNE(n_components=4, verbose=1, perplexity=40, n_iter=300)
        # tsne_results = tsne.fit_transform(interpolated_frame.values)

        # interpolated_frame['tsne-2d-one'] = tsne_results[:,0]
        # interpolated_frame['tsne-2d-two'] = tsne_results[:,1]

        # plt.figure(figsize=(16,10))
        # sn.scatterplot(
        #     x="tsne-2d-one", y="tsne-2d-two",
        #     # hue="y",
        #     palette=sn.color_palette("hls", 10),
        #     data=interpolated_frame,
        #     legend="full",
        #     alpha=0.3
        # )
        # plt.show()


        # ax = plt.figure(figsize=(16,10)).gca(projection='3d')
        # ax.scatter(
        #     xs=df.loc[rndperm,:]["pca-one"],
        #     ys=df.loc[rndperm,:]["pca-two"],
        #     zs=df.loc[rndperm,:]["pca-three"],
        #     c=df.loc[rndperm,:]["y"],
        #     cmap='tab10'
        # )
        # ax.set_xlabel('pca-one')
        # ax.set_ylabel('pca-two')
        # ax.set_zlabel('pca-three')
        # plt.show()

def main(args=None):
    node = Preprocessor()

if __name__ == '__main__':
    main()