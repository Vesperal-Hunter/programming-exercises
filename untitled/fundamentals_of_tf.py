import tensorflow as tf

my_placeholder = tf.placeholder(tf.float32, shape = (100,400))

#########

x = tf.placeholder(tf.float32)
start = tf.Varlable(0.0)
y = start.assign(start + X)


a = [[1,2,3]]
b = [[1,2,3],[4,5,6],[7,8,9]]
c = tf.matmul(a,b)\
sess = tf.Session()
sess.run(c)
writer = tf.train.SummaryWriter('./my_graph', sess.graph)