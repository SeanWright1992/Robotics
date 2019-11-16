class Move(object):
    def __init__(self, Robot, Angles, time):
        #ch0 = (1570 - 10.72 * Angles[0])
        #ch1 = (1450 + 8.33 * Angles[1])  # was 1450
        #ch2 = (1500 - 9.2 * Angles[2])  # 1450 8.33
        #ch3 = (1550 + 7 * Angles[3])

        ch0 = Angles[0]
        ch1 = Angles[1]
        ch2 = Angles[2]
        ch3 = Angles[3]

        out = ("#0 P{} T{} #1 P{} T{} #2 P{} #3 P{}\r".format(ch0, time, ch1, time, ch2, ch3))
        Robot.write(bytes(out, 'utf8'))