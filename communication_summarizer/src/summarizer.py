import rclpy
import signal

from rclpy.node import Node
from communication_trace_msgs.msg import CommunicationTrace

# from -> to
topic_directions = {
    '/communication_trace/sensor_dummy_node_topic1': '/communication_trace/no_dependency_node_topic1',
    '/communication_trace/sensor_dummy_node_topic2': '/communication_trace/sub_dependency_node_topic2',
    '/communication_trace/no_dependency_node_topic3': '/communication_trace/sub_dependency_node_topic3',
    '/communication_trace/sub_dependency_node_topic4': '/communication_trace/timer_dependency_node_topic4',
    '/communication_trace/sub_dependency_node_topic5': '/communication_trace/actuator_dummy_node_topic5',
    '/communication_trace/timer_dependency_node_topic6': '/communication_trace/actuator_dummy_node_topic6',
}

def get_swap_dict(d):
    return {v: k for k, v in d.items()}
topic_rev_directions = get_swap_dict(topic_directions)

class CicularQueue:
    def __init__(self, capacity):
        self.capacity = capacity
        self.queue = [None] * capacity
        self.tail = -1
        self.head = 0
        self.size = 0

    def enqueue(self, item):
        # if self.size == self.capacity:
        #     raise BufferError('Euque is full')

        self.tail =  self.__get_next_idx(self.tail)
        self.queue[self.tail] = item
        # self.size = self.size + 1

    def find(self, key):
        # TODO: speed up to O(1)
        for item in self.queue:
            if item is None:
                continue
            if item.header.stamp.sec == key.sec and item.header.stamp.nanosec == key.nanosec:
                return item
        return None

    # def dequeue(self):
    #     if self.size == 0:
    #         raise BufferError('Euque is empty')

    #     tmp = self.queue[self.head]
    #     self.head = self.__get_next_idx(self.head)
    #     self.size = self.size - 1

    #     return tmp

    def __get_next_idx(self, idx):
        return (idx + 1) % self.capacity

class SummarizeNode(Node):
    # TODO: partition to other class
    def __init__(self, context):
        super().__init__('communication_summarize_node', context=context)
        self.subs_ = []
        self.data_ = {}
        self.timer = self.create_timer(1, self.__update_subscribers) # update every second
        self.time_series_ = {}
        self.stamp_ = {}
        self.communicate_ids_ = [self.__get_communicate_id(_) for _ in topic_directions.keys()]
        for communicate_id in self.communicate_ids_:
            self.time_series_[communicate_id] = []
            self.stamp_[communicate_id] = []

        self.__update_subscribers()

    def export_files(self):
        # TODO: clean
        # TODO: if zero data, print warn message and skip file exporting
        for communicate_id in self.communicate_ids_:
            print(communicate_id)
            with open(communicate_id + '.csv', mode='w') as f:
                for i in range(len(self.time_series_[communicate_id])):
                    f.write('{},{}\n'.format(self.stamp_[communicate_id][i], self.time_series_[communicate_id][i]))

    def __update_subscribers(self):
        topic_names_and_types = super().get_topic_names_and_types()
        for topic_name_and_type in topic_names_and_types:
            topic_name = topic_name_and_type[0]
            type_names = topic_name_and_type[1]

            if not self.__is_trace_topic(topic_name, type_names):
                continue
            if topic_name in self.data_:
                continue

            print('recording {}'.format(topic_name))
            suffix = self.__get_suffix(topic_name)
            self.data_[topic_name] = CicularQueue(capacity = 10)

            sub = self.create_subscription(
                CommunicationTrace,
                topic_name,
                self.create_callback(topic_name, suffix),
                1)
            self.subs_.append(sub)

    def __is_trace_topic(self, topic_name, type_names):
        ns, _ = self.__divide_ns_topic(topic_name)

        return ns == 'communication_trace' and 'communication_trace_msgs/msg/CommunicationTrace' in type_names

    def __get_suffix(self, topic_name):
        idx = topic_name.rfind('_')
        return topic_name[idx+1:]

    def __calc_latency(self, sub_stamp, pub_stamp):
        s = sub_stamp.sec - pub_stamp.sec
        ns = sub_stamp.nanosec - pub_stamp.nanosec
        if ns < 0:
            s = s - 1
            ns = ns + 1e9
        return ns + s*1e9

    def __get_directed_topic_names(self, topic_name):
        if topic_name in topic_directions:
            return topic_name, topic_directions[topic_name]
        if topic_name in topic_rev_directions:
            return topic_rev_directions[topic_name], topic_name

        # TODO:warn
        return "", ""

    def __divide_ns_topic(self, topic_name):
        is_exist_ns = topic_name.count('/') == 2
        if is_exist_ns:
            return topic_name[1:topic_name.rfind('/')], topic_name[topic_name.rfind('/')+1:]
        return "", topic_name[topic_name.rfind('/')+1:]

    def __get_communicate_id(self, topic_name):
        topic_name_from, topic_name_to = self.__get_directed_topic_names(topic_name)

        topic_name_from = topic_name_from[topic_name_from.rfind('/')+1:]
        topic_name_to = topic_name_to[topic_name_to.rfind('/')+1:]

        return '{}->{}'.format(topic_name_from, topic_name_to)

        # TODO: warm log
        return ''

    def create_callback(self, topic_name, suffix):
        import time
        communicate_id = self.__get_communicate_id(topic_name)

        def callback(msg):
            key = msg.header.stamp
            self.data_[topic_name].enqueue(msg)
            if topic_name not in topic_directions.keys() and topic_name not in topic_rev_directions.keys():
                return

            if topic_name in topic_directions:
                related_id = topic_directions[topic_name]

            if topic_name in topic_rev_directions:
                related_id = topic_rev_directions[topic_name]

            if related_id not in self.data_:
                print(related_id, 'is not found.')
                return

            related_msg = self.data_[related_id].find(msg.header.stamp)
            if related_msg == None:
                return
            latency = self.__calc_latency(msg.stamp, related_msg.stamp)

            self.time_series_[communicate_id].append(latency)
            self.stamp_[communicate_id].append(time.time())
            stamp = msg.stamp
        return callback

def main(args=None):
    context = rclpy.context.Context()
    rclpy.init(args=args, context=context)
    node =  SummarizeNode(context=context)

    def shutdown(signum, frame):
        node.export_files()
        context.shutdown()
    signal.signal(signal.SIGINT, shutdown)

    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
