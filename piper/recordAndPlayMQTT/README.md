# Recording and Playing MQTT Messages

## 创建环境变量
```bash
sudo mkdir -p /etc/piper
sudo tee /etc/piper/piper.env > /dev/null <<'EOF'
CAN_IFACE=can0
CAN_BITRATE=1000000
USE_NOHUP=true
EOF
```

## 安装服务
```bash
sudo cp /home/unitree/Agilex-College/piper/recordAndPlayMQTT/piper.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable piper.service
sudo systemctl start piper.service
```

## 查看服务状态
```bash
systemctl status piper.service
journalctl -u piper.service -f

#重启服务
systemctl restart piper.service
```