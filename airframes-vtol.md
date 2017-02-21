# 垂直起降(VTOL)机型 

事实上，[PX4 飞行栈](concept-flight-stack.md) 支持所有类型的垂直起降构型无人机:

  * 尾坐式( X 构型和 十 字构型双旋翼和四旋翼）
  * 倾转旋翼式 (Firefly 的 Y6)
  * 常规固定翼垂直起降飞机(Standard Plane VTOL) (固定翼和四旋翼复合式，下文简称复合式)

垂直起降机型的代码库和其他机型的代码库是一样的，仅仅增加了对于转换模态的控制逻辑。

<aside class="note">
代码中所有的垂直起降构型都已经经过试飞测试，可以直接使用。为了转换过程的安全进行，请确保飞机上装有空速传感器，因为自动驾驶仪在转换模态需要使用空速信号。
</aside>

## 关键设置参数

当创建一个新的飞机构型时，必须正确设置下列参数：

  * `VT_FW_PERM_STAB` 在悬停模式中，自驾仪总是使用姿态稳定模式。如果将该参数设置为1，则飞机默认为姿态稳定模式；如果设置为0，则默认纯手动操纵模式。
  * `VT_ARSP_TRANS` 该参数表示飞机到达多大空速时开始转换到前飞模式，单位为米/秒。若将该参数设置过小，则开始转换时飞机有可能失速。
  * `RC_MAP_TRANS_SW` 该参数为一个切换信号，必须在飞行前指定到遥控器的一个开关通道。这可以使你能够检查多旋翼模态和固定翼模态是否工作正常。（也可以用来在飞行中手动切换多旋翼模态和固定翼模态）

## 尾坐式

[构建日志](airframes-vtol-caipiroshka.md) 包含了更多细节。
{% youtube %}https://www.youtube.com/watch?v=acG0aTuf3f8&vq=hd720{% endyoutube %}

## 倾转旋翼式

 [构建日志](https://pixhawk.org/platforms/vtol/birdseyeview_firefly)包含了获得并运行这些模式的所有设置和说明。 

{% youtube %}https://www.youtube.com/watch?v=Vsgh5XnF44Y&vq=hd720{% endyoutube %}

## 复合式

[构建日志](https://pixhawk.org/platforms/vtol/fun_cub_quad_vtol)包含了构建并复现如下视频的进一步说明。 

{% youtube %}https://www.youtube.com/watch?v=4K8yaa6A0ks&vq=hd720{% endyoutube %}

{% youtube %}https://www.youtube.com/watch?v=7tGXkW6d3sA&vq=hd720{% endyoutube %}

