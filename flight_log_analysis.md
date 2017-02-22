# 飞行日志分析

存在很多软件包可以分析 PX4 的飞行日志。

## [Log Muncher](http://logs.uaventure.com)

说明：Log Muncher 只能适用于之前的日志格式 (.px4log)。

### 上传

用户可以访问这个网页和直接上传日志文件：[http://logs.uaventure.com/](http://logs.uaventure.com/)

![](images/flight_log_analysis/logmuncher.png)

### 结果

![](images/flight_log_analysis/log-muncher-result.png)

[Example Log](http://logs.uaventure.com/view/KwTFDaheRueMNmFRJQ3huH)

### 优点
* 基于web，非常适合终端用户  
* 用户可以上传，加载和分享报告给其他用户

### 缺点
* 分析受到非常大的约束，不可能定制。


## [Flight Review](http://logs.px4.io)

Flight Review 是 Log Muncher 的继承者，能够结合新的 ULog 日志格式使用。

### 示例
![](images/flight_log_analysis/flight-review-example.png)

### 优点
* 基于 web，非常适合终端用户
* 用户可以上传，加载和分享报告给其他用户
* 交互的绘图

### 缺点
* 不可能定制


## [FlightPlot](https://github.com/PX4/FlightPlot)

![](https://pixhawk.org/_media/dev/flightplot-0.2.16-screenshot.png)

* [FlightPlot 下载](https://s3.amazonaws.com/flightplot/releases/latest.html)

### 优点
* 基于 java，跨平台
* 非常直观的 GUI，无需任何编程知识

### 缺点
* 根据已经内建的特征约束分析

## [PX4Tools](https://github.com/dronecrew/px4tools)

![](images/flight_log_analysis/px4tools.png)

### 安装

* 推荐步骤是使用 anaconda3。查看 [px4tools github page](https://github.com/dronecrew/px4tools) 获取更多细节。

```bash
conda install -c https://conda.anaconda.org/dronecrew px4tools
```

### 优点
* 能够轻易地分享，用户可以查看 github 上的 notebooks (例如 [https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb](https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb))
* 基于 python，跨平台，使用 anaconda 2 和 anaconda3
* ipython 或 jupyter notebooks 能够轻易地分享分析
* 更先进的绘图功能允许了细节上的分析

### 缺点
* 要求用户了解 python
* 目前需要在使用之前利用 sdlog2_dump.py 或嵌入 px42csv 程序的 px4tools 将日志文件转换为 csv文件
