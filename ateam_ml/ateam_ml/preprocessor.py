import os
import shutil
import pandas as pd
import seaborn as sn
import matplotlib.pyplot as plt

from rclpy.node import Node
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py import message_to_ordereddict

from rclpy.parameter import Parameter

from IPython.display import display

from sklearn.decomposition import PCA
from sklearn.manifold import TSNE

import rosbag2_py

class Preprocessor(Node):
    def __init__(self):
        super().__init__("Preprocessor", automatically_declare_parameters_from_overrides=True)
        self.reader = rosbag2_py.SequentialReader()

        param_bag_path = self.get_parameter('bag_path').value
        print(f"Path for bag {param_bag_path}")
        bag_dir = os.path.dirname(param_bag_path)

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=param_bag_path,
            storage_id='')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        topics_meta = self.reader.get_all_topics_and_types()
        topics_table = {topic_meta.name : get_message(topic_meta.type) for topic_meta in topics_meta}

        excluded_topic_list = ['vision_messages', 'uiclient_count', 'rosout', 'vision_messages', 'ui_connected_clients.csv', 'parameter_events']
        topic_list = [topic for topic in topics_table if topic not in excluded_topic_list]


        # topic_list = [
        #     'ball',
        #     'blue_teamrobot1',
        #     # 'blue_teamrobot2',
        #     'blue_teamrobot3',
        #     # 'blue_teamrobot4',
        #     'blue_teamrobot5',
        #     # 'yellow_teamrobot1',
        #     'yellow_teamrobot2',
        #     # 'yellow_teamrobot3',
        #     'yellow_teamrobot4',
        #     'yellow_teamrobot5',
        #     'yellow_teamrobot6',
        #     'yellow_teamrobot7',
        #     # 'yellow_teamrobot9'
        # ]

        csv_dir = os.path.join(bag_dir, "csvs")

        mainframe = pd.DataFrame()

        for topic in topic_list:
            csv_path = os.path.join(csv_dir, topic + ".csv")
            dataframe = pd.read_csv(csv_path)
            dataframe = dataframe.dropna(axis='index', how='any')
            dataframe['timestamp'] = pd.to_datetime(dataframe['timestamp'])
            dataframe = dataframe.set_index('timestamp')
            dataframe = dataframe.add_prefix(f'{topic}.')

            mainframe = pd.concat([mainframe, dataframe], axis=1, join='outer')

        print("final mainframe:\n ", mainframe)

        # gives a good idea of what you can reasonably interpolate
        times = pd.DataFrame({"times": mainframe.index.values.astype(float)})
        diffs_in_seconds = (times['times'].diff() / 10**9)
        print(diffs_in_seconds.describe())

        upsampled = mainframe.resample(pd.Timedelta('1milli')).mean()
        # upsampled = mainframe.resample(pd.Timedelta('10milli')).mean()

        # interpolated_frame = upsampled.interpolate(method='linear')
        # interpolated_frame = upsampled.interpolate(method='barycentric') # this one takes a small enternity
        interpolated_frame = upsampled.interpolate(method='akima')

        interpolated_frame = interpolated_frame.dropna(axis='index', how='any')

        print("interpolated_frame:\n ", interpolated_frame)
        # print(interpolated_frame.count())


        # comparison of pre interpolation to after
        # wipe = lambda df, topic: df.dropna(axis='index', how='any', subset=topic)[topic].values
        # plt.plot(wipe(interpolated_frame, "ball.pose.position.x"), wipe(interpolated_frame,"ball.pose.position.y"), 'bo', ms=1)
        # plt.plot(wipe(mainframe, "ball.pose.position.x"), wipe(mainframe,"ball.pose.position.y"), 'go', ms=1)
        # plt.show()


        tsne = TSNE(n_components=4, verbose=1, perplexity=40, n_iter=300)
        tsne_results = tsne.fit_transform(interpolated_frame.values)

        interpolated_frame['tsne-2d-one'] = tsne_results[:,0]
        interpolated_frame['tsne-2d-two'] = tsne_results[:,1]

        plt.figure(figsize=(16,10))
        sn.scatterplot(
            x="tsne-2d-one", y="tsne-2d-two",
            # hue="y",
            palette=sn.color_palette("hls", 10),
            data=interpolated_frame,
            legend="full",
            alpha=0.3
        )
        plt.show()


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
    rclpy.init(args=args)
    node = Preprocessor()
    rclpy.shutdown()

if __name__ == '__main__':
    main()