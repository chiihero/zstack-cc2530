import matplotlib.pyplot as plt

def plt_init():

    plt.ion()  # 开启interactive mode 成功的关键函数
    plt.rcParams['figure.figsize'] = (5, 5)  # 图像显示大小
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 防止中文标签乱码，还有通过导入字体文件的方法
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['lines.linewidth'] = 0.5  # 设置曲线线条宽度
    plt.figure(1,clear=True)



def pltstart():
    plt.clf()  # 清除刷新前的图表，防止数据量过大消耗内存
    # plt.suptitle("监控", fontsize=30)  # 添加总标题，并设置文字大小

def pltpause():
    plt.pause(2)


def pltset(num, title, ylab, x, y,y1=None,y2=None):
    if num ==0 or num ==7:
        plt.subplot(1, 1, 1)
    else:
        plt.subplot(3, 2, num)
    plt.title(title)
    plt.text(0, 0, '当前值=' + checkstr(ylab, y[-1]), fontsize=20)
    plt.xlabel('时间', fontsize=10)
    plt.ylabel(ylab, fontsize=15)
    plt.plot(x, y, 'g-')
    if num==6 or num ==7:
        plt.plot(x, y1, color='red')
        plt.plot(x, y2, color='blue')

def checkstr(ylab, y):
    # 针对人体红外传感器
    if ylab == "有人":
        if y == 1:
            return "有人"
        else:
            return "没人"
    else:
        return str(y)
