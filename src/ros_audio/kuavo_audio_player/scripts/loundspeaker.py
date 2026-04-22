#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import queue
import os
try:    
    import pyaudio
except ImportError:
    print("pyaudio 未安装，先安装 pyaudio")
    command = "sudo apt-get install python3-pyaudio -y"
    os.system(command)  
    import pyaudio
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from kuavo_msgs.srv import playmusic, playmusicResponse
import subprocess
import time
import signal
from scipy import signal
try:
    import samplerate
except ImportError:
    print("samplerate 未安装，先安装 samplerate")
    command = "pip install samplerate -i https://mirrors.aliyun.com/pypi/simple/ --no-input"
    os.system(command)
    import samplerate
audio_path = "/home/lab/.config/lejuconfig/music"

class MusicPlayerNode:
    def __init__(self):
        while not self.check_sound_card():
            print("未检测到播音设备，不启用播音功能！")
            time.sleep(10000)   

        rospy.init_node('music_player_node')
        self.service = rospy.Service('play_music', playmusic, self.play_music_callback)
        audio_path = rospy.get_param('music_path')
        self.music_directory = audio_path
        self.audio_subscriber = rospy.Subscriber('audio_data', Int16MultiArray, self.audio_callback, queue_size=10)
        self.stop_music_subscriber = rospy.Subscriber('stop_music', Bool, self.stop_music_callback, queue_size=10)   
        rospy.loginfo("已创建 audio_data 话题的订阅者")

        # 初始化 PyAudio 播放
        self.chunk_size = 8192
        self.buffer_queue = queue.Queue(maxsize=50)  # 限制最大缓冲块数
        self.playing = True
        self.empty_count = 0
        self.p = pyaudio.PyAudio()
        # 获取声卡默认采样率
        try:
            device_info = self.p.get_default_output_device_info()
            self.rate = int(device_info.get('defaultSampleRate', 16000))
            rospy.loginfo(f"检测到声卡默认采样率: {self.rate}Hz")
        except Exception as e:
            rospy.logwarn(f"无法获取声卡默认采样率，使用默认值16000Hz: {e}")
            self.rate = 16000
        self.channels = 1
        self.current_music_process = []  # 记录当前播放音乐的进程
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=self.channels,
                                  rate=self.rate,
                                  output=True,
                                  frames_per_buffer=self.chunk_size)

        # 播放线程
        self.play_thread = threading.Thread(target=self.play_from_buffer)
        self.play_thread.daemon = True
        self.play_thread.start()

    def resample_audio(self, audio_chunk):
        try:
            # 将 int16 数据转换为 float32 范围 [-1, 1]
            audio_chunk = audio_chunk.astype(np.float32) / 32768.0
            resample_ratio = self.rate / 16000.0
            audio_chunk = samplerate.resample(audio_chunk, resample_ratio, converter_type='sinc_fastest')
            # 重新转为 int16
            audio_chunk = np.clip(audio_chunk * 32768.0, -32768, 32767).astype(np.int16)
            return audio_chunk
        except Exception as e:
            rospy.logerr(f"音频重采样失败: {e}")
            return np.zeros(self.chunk_size, dtype=np.int16)

    def set_rate(self, new_rate):
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()

        self.rate = new_rate
        self.stream = self.p.open(format=pyaudio.paInt16,
                                channels=1,
                                rate=self.rate,
                                output=True,
                                frames_per_buffer=self.chunk_size)


    def audio_callback(self, msg):
        try:
            # rospy.loginfo(f"收到音频块，大小: {len(msg.data)}")
            audio_chunk = np.array(msg.data, dtype=np.int16)
            if len(audio_chunk) < self.chunk_size and self.buffer_queue.qsize() == 0:
                rospy.loginfo(f"音频块大小不足，补齐")
                audio_chunk = np.concatenate((audio_chunk, np.zeros(self.chunk_size - len(audio_chunk), dtype=np.int16))) 
            audio_chunk = self.resample_audio(audio_chunk)
            self.buffer_queue.put(audio_chunk, timeout=1)  # 超时避免卡死
        except queue.Full:
            rospy.logwarn("音频缓冲区已满，丢弃音频块")
        except Exception as e:
            rospy.logerr(f"添加到缓冲区失败: {e}")

    def play_from_buffer(self):
        while not rospy.is_shutdown():
            try:
                chunk = self.buffer_queue.get(timeout=1)
                self.stream.write(chunk.tobytes())
            except queue.Empty:
                if(self.empty_count > 100):
                    rospy.logwarn("缓冲区为空，等待音频输入")
                    self.empty_count = 0
                self.empty_count += 1
            except Exception as e:
                rospy.logerr(f"播放缓冲区音频失败: {e}")


    def check_sound_card(self):
        """
        检查声卡状态，特别是耳机和扬声器的可用性
        """
        # 检查耳机状态
        try:
            headphone_command = 'pactl list | grep -i Headphone'
            headphone_result = subprocess.run(headphone_command, shell=True, capture_output=True, text=True)
            print(headphone_result.stdout)
            if not bool(headphone_result.stdout.strip()):
                print(f"不存在耳机设备")
            # 检查耳机是否不可用
            else:
                headphone_available = "not available" not in headphone_result.stdout
                print(f"耳机状态: {'可用' if headphone_available else '不可用'}")
                if headphone_available:
                    return True
            # 检查扬声器状态
            speaker_command = 'pactl list | grep -i Speaker'
            speaker_result = subprocess.run(speaker_command, shell=True, capture_output=True, text=True)
            
            # 检查扬声器是否存在
            speaker_exists = bool(speaker_result.stdout.strip())
            print(f"扬声器状态: {'存在' if speaker_exists else '不存在'}")
            if speaker_exists:
                return True
            
            # root用户下检查扬声器状态
            root_speaker_command = 'aplay -l | grep -i Audio'
            root_speaker_result = subprocess.run(root_speaker_command, shell=True, capture_output=True, text=True)
            print(root_speaker_result.stdout)
            root_speaker_exists = bool(root_speaker_result.stdout.strip())
            print(f"root扬声器状态: {'存在' if root_speaker_exists else '不存在'}")
            if not root_speaker_exists:
                print(f"不存在扬声器设备")
            else:
                return True
            
            return False
            
        except Exception as e:
            print(f"检查声卡状态时出错: {str(e)}")
            return False

    def play_music_callback(self, req):
        try:   
            music_file = os.path.join(self.music_directory, f"{req.music_number}")
            play_command = ['play', '-q', music_file,'vol', str(req.volume), 'speed', str(req.speed)]
            self.current_music_process.append(subprocess.Popen(play_command))
            rospy.loginfo(f"播放本地音乐 {music_file}，音量 {req.volume}%，速度 {req.speed}倍，进程ID: {self.current_music_process[-1].pid}")
            return playmusicResponse(success_flag=True)
        except Exception as e:
            rospy.logerr(f"播放音乐出错: {str(e)}")
            return playmusicResponse(success_flag=False)

    def stop_music_callback(self, msg):
        """
        停止当前正在播放的音乐
        """
        if msg.data:
            try:
                # 如果有记录的音乐进程，直接终止该进程
                if self.current_music_process is not None:
                    for process in self.current_music_process:
                        if process.poll() is None:  # 检查进程是否仍在运行
                           kill_command = ['kill', '-9', str(process.pid)]
                           subprocess.call(kill_command)    
                           rospy.loginfo(f"已停止音乐进程 PID: {process.pid}")
                    self.current_music_process = []
                else:
                    # 使用系统命令停止所有正在播放的音乐（备用方法）
                    stop_command = ['killall', 'play']
                    subprocess.call(stop_command)
                    rospy.loginfo("已停止所有正在播放的音乐")
                
                self.buffer_queue.queue.clear()
                return True
            except Exception as e:
                rospy.logerr(f"停止音乐时出错: {str(e)}")
                return False

    def shutdown(self):
        self.playing = False
        # 停止可能正在运行的音乐进程
        if self.current_music_process is not None:
            try:
                os.kill(self.current_music_process.pid, signal.SIGTERM)
            except:
                pass
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        rospy.loginfo("播放已停止，资源已释放")

    def run(self):
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    player_node = MusicPlayerNode()
    player_node.run()
