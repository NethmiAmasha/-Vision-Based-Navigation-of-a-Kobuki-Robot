import movement
from kobukidriversample import Kobuki
import colordetectionwhilerotatng
import navigate


color_sequence = ['red', 'blue', 'green', 'yellow']

kobuki = Kobuki()


# x=colordetectionwhilerotatng.find_color_boxes_between_lines(kobuki,color_sequence,100)
colordetectionwhilerotatng.find_color_boxes(kobuki,color_sequence,100)


navigate.robot_navigation(
    kobuki=kobuki,
    color="white",
    y_center=300,
    gap=50,
    forward_speed=100
)



navigate.robot_navigation(
    kobuki=kobuki,
    color="red",
    y_center=400,
    gap=50,
    forward_speed=100
)


