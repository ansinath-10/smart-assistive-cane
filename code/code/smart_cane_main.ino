#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include <math.h>

// ============================================================
//  WiFi credentials
// ============================================================
const char* WIFI_SSID     = "Nothing Phone";
const char* WIFI_PASSWORD = "1234567890";

// ============================================================
//  Pin definitions
// ============================================================
#define TRIG_PIN        38
#define ECHO_PIN        14
#define VIBRATION_PIN    2
#define BUZZER_PIN       1

// ============================================================
//  Camera pins (Freenove ESP32-S3-CAM)
// ============================================================
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y2_GPIO_NUM     11
#define Y3_GPIO_NUM      9
#define Y4_GPIO_NUM      8
#define Y5_GPIO_NUM     10
#define Y6_GPIO_NUM     12
#define Y7_GPIO_NUM     18
#define Y8_GPIO_NUM     17
#define Y9_GPIO_NUM     16
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13

// ============================================================
//  Saved GPS locations
// ============================================================
struct Location {
  const char* name;
  const char* label;
  double lat;
  double lng;
};

const Location SAVED_LOCATIONS[] = {
  { "campus",            "Campus Cham",        49.489200, 12.662700 },
  { "maximilianstrasse", "Maximilianstrasse",   49.453500, 11.038300 },
  { "home",              "Home",                0.000000,  0.000000  }
};
const int NUM_LOCATIONS = 3;

// ============================================================
//  Globals
// ============================================================
WebServer      server(80);
httpd_handle_t stream_httpd = NULL;

float         currentDistance   = 999.0;
unsigned long lastDistanceMs    = 0;
unsigned long lastAlertMs       = 0;
bool          cameraOK          = false;

double        phoneLat          = 0.0;
double        phoneLng          = 0.0;
bool          phoneGpsFix       = false;
double        prevLat           = 0.0;
double        prevLng           = 0.0;

bool          navigating        = false;
int           navTarget         = -1;

// ============================================================
//  Maths
// ============================================================
double toRad(double d) { return d * M_PI / 180.0; }

double haversineDistance(double la1,double lo1,double la2,double lo2){
  double dLat=toRad(la2-la1),dLon=toRad(lo2-lo1);
  double a=sin(dLat/2)*sin(dLat/2)+cos(toRad(la1))*cos(toRad(la2))*sin(dLon/2)*sin(dLon/2);
  return 6371000.0*2*atan2(sqrt(a),sqrt(1-a));
}

double bearingTo(double la1,double lo1,double la2,double lo2){
  double dLon=toRad(lo2-lo1);
  double y=sin(dLon)*cos(toRad(la2));
  double x=cos(toRad(la1))*sin(toRad(la2))-sin(toRad(la1))*cos(toRad(la2))*cos(dLon);
  return fmod((atan2(y,x)*180.0/M_PI+360.0),360.0);
}

String compassDir(double b){
  if(b<22.5)  return "north";
  if(b<67.5)  return "northeast";
  if(b<112.5) return "east";
  if(b<157.5) return "southeast";
  if(b<202.5) return "south";
  if(b<247.5) return "southwest";
  if(b<292.5) return "west";
  if(b<337.5) return "northwest";
  return "north";
}

// ============================================================
//  Camera
// ============================================================
bool setupCamera(){
  camera_config_t config;
  config.ledc_channel=LEDC_CHANNEL_0; config.ledc_timer=LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM;
  config.pin_d2=Y4_GPIO_NUM; config.pin_d3=Y5_GPIO_NUM;
  config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM;
  config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM;
  config.pin_vsync=VSYNC_GPIO_NUM; config.pin_href=HREF_GPIO_NUM;
  config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM;
  config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz=10000000; config.pixel_format=PIXFORMAT_JPEG;
  config.frame_size=FRAMESIZE_QVGA; config.jpeg_quality=15;
  config.fb_count=2; config.fb_location=CAMERA_FB_IN_PSRAM;
  config.grab_mode=CAMERA_GRAB_LATEST;
  esp_err_t err=esp_camera_init(&config);
  if(err!=ESP_OK){ Serial.printf("Camera FAILED: 0x%x\n",err); return false; }
  Serial.println("Camera OK"); return true;
}

// ============================================================
//  Camera stream on port 81
// ============================================================
#define PART_BOUNDARY "smartcaneboundary"
static const char* STREAM_CONTENT_TYPE="multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY="\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART="Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req){
  if(!cameraOK) return ESP_FAIL;
  httpd_resp_set_type(req,STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  char part_buf[128];
  while(true){
    camera_fb_t *fb=esp_camera_fb_get();
    if(!fb) break;
    size_t hlen=snprintf(part_buf,sizeof(part_buf),STREAM_PART,fb->len);
    esp_err_t res=httpd_resp_send_chunk(req,STREAM_BOUNDARY,strlen(STREAM_BOUNDARY));
    if(res==ESP_OK) res=httpd_resp_send_chunk(req,part_buf,hlen);
    if(res==ESP_OK) res=httpd_resp_send_chunk(req,(const char*)fb->buf,fb->len);
    esp_camera_fb_return(fb);
    if(res!=ESP_OK) break;
    delay(50);
  }
  return ESP_OK;
}

void startCameraServer(){
  httpd_config_t config=HTTPD_DEFAULT_CONFIG();
  config.server_port=81; config.ctrl_port=32769;
  httpd_uri_t stream_uri={"/stream",HTTP_GET,stream_handler,NULL};
  if(httpd_start(&stream_httpd,&config)==ESP_OK){
    httpd_register_uri_handler(stream_httpd,&stream_uri);
    Serial.println("Stream server on port 81");
  }
}

// ============================================================
//  Ultrasonic
// ============================================================
float measureDistanceCm(){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  long d=pulseIn(ECHO_PIN,HIGH,30000);
  return d==0?999.0:d*0.034f/2.0f;
}

// ============================================================
//  Buzzer & vibration
// ============================================================
void beep(int n,int ms){
  for(int i=0;i<n;i++){
    digitalWrite(BUZZER_PIN,HIGH); delay(ms);
    digitalWrite(BUZZER_PIN,LOW);
    if(i<n-1) delay(80);
  }
}
void vibrate(int n,int ms){
  for(int i=0;i<n;i++){
    digitalWrite(VIBRATION_PIN,HIGH); delay(ms);
    digitalWrite(VIBRATION_PIN,LOW);
    if(i<n-1) delay(80);
  }
}

// ============================================================
//  Handlers
// ============================================================

// ── NEW: /capture — single JPEG snapshot on port 80 ──────────
// This fixes the cross-origin canvas block that broke detection.
// Port 81 stream cannot be drawn to canvas (cross-origin taint).
// This endpoint is same-origin so COCO-SSD can read the pixels.
void handleCapture(){
  if(!cameraOK){
    server.sendHeader("Access-Control-Allow-Origin","*");
    server.send(500,"text/plain","no camera");
    return;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if(!fb){
    server.sendHeader("Access-Control-Allow-Origin","*");
    server.send(500,"text/plain","capture failed");
    return;
  }
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Cache-Control","no-cache, no-store");
  server.sendHeader("Pragma","no-cache");
  server.send_P(200,"image/jpeg",(const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Phone sends its GPS location here every 2 seconds
void handleGpsUpdate(){
  if(server.hasArg("lat") && server.hasArg("lng")){
    double newLat=server.arg("lat").toDouble();
    double newLng=server.arg("lng").toDouble();
    if(newLat!=0 && newLng!=0){
      if(phoneLat!=0){ prevLat=phoneLat; prevLng=phoneLng; }
      phoneLat=newLat; phoneLng=newLng; phoneGpsFix=true;
    }
  }
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.send(200,"text/plain","ok");
}

void handleStatus(){
  String navJson="{\"active\":false}";
  if(navigating && navTarget>=0){
    if(phoneGpsFix){
      double tLat=SAVED_LOCATIONS[navTarget].lat;
      double tLng=SAVED_LOCATIONS[navTarget].lng;
      double dist=haversineDistance(phoneLat,phoneLng,tLat,tLng);
      double bear=bearingTo(phoneLat,phoneLng,tLat,tLng);
      String turn="unknown";
      if(prevLat!=0){
        double mb=bearingTo(prevLat,prevLng,phoneLat,phoneLng);
        double diff=bear-mb;
        if(diff>180) diff-=360; if(diff<-180) diff+=360;
        if(diff>20) turn="right"; else if(diff<-20) turn="left"; else turn="straight";
      }
      navJson="{\"active\":true,\"target\":\""+String(SAVED_LOCATIONS[navTarget].label)+
              "\",\"distance\":"+String(dist,0)+
              ",\"bearing\":"+String(bear,1)+
              ",\"compass\":\""+compassDir(bear)+
              "\",\"turn\":\""+turn+"\"}";
    } else {
      navJson="{\"active\":true,\"target\":\""+String(SAVED_LOCATIONS[navTarget].label)+"\",\"waiting\":true}";
    }
  }
  String json="{\"distance\":"+String(currentDistance,1)+
              ",\"lat\":"+String(phoneLat,6)+
              ",\"lng\":"+String(phoneLng,6)+
              ",\"gps_fix\":"+(phoneGpsFix?"true":"false")+
              ",\"camera\":"+(cameraOK?"true":"false")+
              ",\"nav\":"+navJson+"}";
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Cache-Control","no-cache");
  server.send(200,"application/json",json);
}

void handleNav(){
  if(!server.hasArg("cmd")){ server.send(400,"text/plain","Missing cmd"); return; }
  String cmd=server.arg("cmd"); cmd.toLowerCase();
  if(cmd=="stop"){ navigating=false; navTarget=-1; server.send(200,"text/plain","stopped"); return; }
  for(int i=0;i<NUM_LOCATIONS;i++){
    if(cmd==String(SAVED_LOCATIONS[i].name)){
      if(SAVED_LOCATIONS[i].lat==0.0){ server.send(400,"text/plain","Location coords not set"); return; }
      navigating=true; navTarget=i; prevLat=0; prevLng=0;
      beep(2,150);
      server.send(200,"text/plain","Navigating to "+String(SAVED_LOCATIONS[i].label));
      return;
    }
  }
  server.send(400,"text/plain","Unknown: "+cmd);
}

void handleAlert(){
  if(server.hasArg("level")){
    int l=server.arg("level").toInt();
    if(l==3){ vibrate(3,300); beep(3,300); }
    else if(l==2){ vibrate(2,200); beep(2,200); }
    else if(l==1){ vibrate(1,150); beep(1,150); }
  }
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.send(200,"text/plain","ok");
}

// ============================================================
//  Main UI
// ============================================================
void handleRoot(){
  String html = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta charset="UTF-8">
<title>Smart Cane</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee}
h1{text-align:center;padding:12px;background:#16213e;color:#00d4ff;font-size:18px;letter-spacing:2px}
#stream-box{width:100%;background:#000}
#stream-box img{width:100%;max-width:480px;display:block;margin:0 auto}
.cards{display:grid;grid-template-columns:1fr 1fr;gap:8px;padding:8px}
.card{background:#16213e;border-radius:10px;padding:12px;text-align:center;border:1px solid #0f3460}
.lbl{font-size:10px;color:#888;text-transform:uppercase;letter-spacing:1px;margin-bottom:4px}
.val{font-size:26px;font-weight:bold;color:#00ff88}
.val.warn{color:#ffaa00}.val.danger{color:#ff4444}
.unit{font-size:11px;color:#555;margin-top:2px}
.wide{grid-column:span 2;background:#16213e;border-radius:10px;padding:12px;border:1px solid #0f3460}
#nav-info{font-size:14px;color:#00d4ff;min-height:36px;line-height:1.5;margin-top:4px}
#obj-name{font-size:14px;color:#00ff88;margin-top:4px;min-height:20px}
#coords{font-size:12px;color:#00d4ff;margin-top:4px}
#map-link{display:none;color:#00aaff;font-size:12px;margin-top:4px}
.sec{padding:0 8px 8px}
.sec h3{font-size:10px;color:#888;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px}
.loc-btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.loc-btn{padding:11px 4px;border:none;border-radius:10px;font-size:12px;font-weight:bold;
         cursor:pointer;background:#0f3460;color:#fff}
.loc-btn.active{background:#00448a;border:2px solid #00d4ff}
#stop-btn{width:100%;padding:11px;border:none;border-radius:10px;font-size:13px;
          font-weight:bold;cursor:pointer;background:#6b0000;color:#fff;margin-top:6px}
#voice-btn{width:100%;padding:13px;border:none;border-radius:10px;font-size:14px;
           font-weight:bold;cursor:pointer;background:#004d2e;color:#fff;margin-bottom:6px}
#voice-btn.listening{background:#006b3a;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}
#vstat{text-align:center;font-size:11px;color:#888;margin-bottom:6px}
#sbar{text-align:center;font-size:10px;color:#444;padding:6px}
#gps-status{text-align:center;font-size:11px;padding:4px 8px;margin:0 8px 4px;
            border-radius:6px;background:#0f3460}
#det-status{font-size:10px;color:#555;margin-top:3px;text-align:center}
</style>
</head>
<body>
<h1>🦯 SMART CANE</h1>

<div id="stream-box">
  <img id="cam" alt="Camera feed" />
</div>

<!-- Hidden canvas used for AI detection frames -->
<canvas id="det-canvas" style="display:none"></canvas>

<!-- GPS status banner -->
<div id="gps-status">📍 Requesting phone GPS...</div>

<div class="cards">
  <div class="card">
    <div class="lbl">Distance</div>
    <div class="val" id="dist">--</div>
    <div class="unit">cm</div>
  </div>
  <div class="card">
    <div class="lbl">GPS Fix</div>
    <div class="val" id="fix-val" style="font-size:20px">--</div>
  </div>

  <div class="wide">
    <div class="lbl">👀 What's Ahead</div>
    <div id="obj-name">Loading AI detector...</div>
    <div id="det-status"></div>
  </div>

  <div class="wide">
    <div class="lbl">🧭 Navigation</div>
    <div id="nav-info">Not navigating</div>
  </div>

  <div class="wide">
    <div class="lbl">📍 My Location</div>
    <div id="coords">Waiting for GPS...</div>
    <a id="map-link" href="#" target="_blank">Open in Google Maps ↗</a>
  </div>
</div>

<div class="sec">
  <h3>🎤 Voice Command</h3>
  <button id="voice-btn" onclick="startVoice()">🎤 Tap &amp; Speak</button>
  <div id="vstat">Say: "go to campus" · "go to maximilianstrasse" · "go home" · "stop"</div>
</div>

<div class="sec">
  <h3>Navigate To</h3>
  <div class="loc-btns">
    <button class="loc-btn" id="btn-campus"            onclick="navTo('campus')">🏫 Campus</button>
    <button class="loc-btn" id="btn-maximilianstrasse" onclick="navTo('maximilianstrasse')">🚇 Maxim.</button>
    <button class="loc-btn" id="btn-home"              onclick="navTo('home')">🏠 Home</button>
  </div>
  <button id="stop-btn" onclick="stopNav()">⏹ Stop Navigation</button>
</div>

<div id="sbar">Connecting...</div>

<!-- TensorFlow + COCO-SSD -->
<script src="https://cdnjs.cloudflare.com/ajax/libs/tensorflow/4.10.0/tf.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.3/dist/coco-ssd.min.js"></script>

<script>
// ── Stream on port 81 (display only) ────────────────────────
document.getElementById('cam').src =
  window.location.protocol + '//' + window.location.hostname + ':81/stream';

// ── Speech ────────────────────────────────────────────────────
const synth = window.speechSynthesis;
let lastSpoken = '', lastSpokenTime = 0;
function speak(text, force){
  const now = Date.now();
  if(!force && text === lastSpoken && now - lastSpokenTime < 8000) return;
  synth.cancel();
  const u = new SpeechSynthesisUtterance(text);
  u.lang = 'en-US'; u.rate = 0.95; u.pitch = 1.0;
  synth.speak(u);
  lastSpoken = text; lastSpokenTime = now;
}

// ── Phone GPS ─────────────────────────────────────────────────
let phoneGpsOK = false;
let lastGpsSend = 0;

function startPhoneGPS(){
  if(!navigator.geolocation){
    document.getElementById('gps-status').textContent = '❌ GPS not supported';
    document.getElementById('gps-status').style.color = '#ff4444';
    return;
  }
  navigator.geolocation.watchPosition(
    pos => {
      const lat = pos.coords.latitude;
      const lng = pos.coords.longitude;
      const acc = pos.coords.accuracy;
      phoneGpsOK = true;
      document.getElementById('gps-status').textContent =
        '✅ Phone GPS active — accuracy: ' + Math.round(acc) + 'm';
      document.getElementById('gps-status').style.color = '#00ff88';
      const now = Date.now();
      if(now - lastGpsSend > 2000){
        fetch('/gps?lat=' + lat + '&lng=' + lng).catch(() => {});
        lastGpsSend = now;
      }
    },
    err => {
      let msg = '❌ GPS error: ';
      if(err.code === 1)      msg += 'Permission denied — allow location in browser';
      else if(err.code === 2) msg += 'Position unavailable — go outdoors';
      else                    msg += 'Timeout — move to open area';
      document.getElementById('gps-status').textContent = msg;
      document.getElementById('gps-status').style.color = '#ffaa00';
    },
    { enableHighAccuracy: true, maximumAge: 2000, timeout: 10000 }
  );
}

// ── Voice recognition ─────────────────────────────────────────
let recognition = null;
if('webkitSpeechRecognition' in window || 'SpeechRecognition' in window){
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  recognition = new SR();
  recognition.lang = 'en-US';
  recognition.continuous = false;
  recognition.interimResults = false;
  recognition.onresult = e => {
    const cmd = e.results[0][0].transcript.toLowerCase().trim();
    document.getElementById('vstat').textContent = 'Heard: "' + cmd + '"';
    handleVoiceCmd(cmd);
    document.getElementById('voice-btn').classList.remove('listening');
  };
  recognition.onerror = e => {
    document.getElementById('vstat').textContent = 'Error: ' + e.error;
    document.getElementById('voice-btn').classList.remove('listening');
  };
  recognition.onend = () => document.getElementById('voice-btn').classList.remove('listening');
}

function startVoice(){
  if(!recognition){ speak('Voice not supported', true); return; }
  document.getElementById('voice-btn').classList.add('listening');
  document.getElementById('vstat').textContent = 'Listening...';
  recognition.start();
}

function handleVoiceCmd(cmd){
  if(cmd.includes('campus'))
    navTo('campus');
  else if(cmd.includes('maximilianstrasse') || cmd.includes('maximilian'))
    navTo('maximilianstrasse');
  else if(cmd.includes('home'))
    navTo('home');
  else if(cmd.includes('stop'))
    stopNav();
  else
    speak('Command not recognised. Say go to campus, go home, or stop.', true);
}

// ── Navigation ────────────────────────────────────────────────
function navTo(loc){
  fetch('/nav?cmd=' + loc).then(r => r.text()).then(() => {
    speak('Navigating to ' + loc.replace('maximilianstrasse','Maximilian strasse'), true);
    document.querySelectorAll('.loc-btn').forEach(b => b.classList.remove('active'));
    const b = document.getElementById('btn-' + loc);
    if(b) b.classList.add('active');
  });
}

function stopNav(){
  fetch('/nav?cmd=stop').then(() => {
    speak('Navigation stopped', true);
    document.querySelectorAll('.loc-btn').forEach(b => b.classList.remove('active'));
  });
}

// ── Object detection ──────────────────────────────────────────
// FIX: Uses /capture (port 80, same-origin) instead of drawing
// the :81 stream to canvas — that caused a cross-origin taint
// which made every frame blank and detection never fired.
let detector    = null;
let lastDetLabel  = '';
let lastDetSpeak  = 0;
let currentDist   = 999;
let detRunning    = false;

async function loadDetector(){
  try {
    document.getElementById('obj-name').textContent = '⏳ Loading AI model...';
    detector = await cocoSsd.load();
    document.getElementById('obj-name').textContent = 'AI ready — watching...';
    document.getElementById('det-status').textContent = 'Model loaded ✅';
    detectLoop();
  } catch(e) {
    document.getElementById('obj-name').textContent = 'Detector unavailable';
    document.getElementById('det-status').textContent = 'Error: ' + e.message;
  }
}

async function detectLoop(){
  if(detRunning){ setTimeout(detectLoop, 1000); return; }
  detRunning = true;

  if(detector){
    try {
      // Fetch a fresh JPEG snapshot from the same origin (port 80)
      // This avoids the cross-origin taint that blocked canvas access
      const snapUrl = '/capture?t=' + Date.now();

      const snapImg = await new Promise((resolve, reject) => {
        const img = new Image();
        img.onload  = () => resolve(img);
        img.onerror = () => reject(new Error('Snapshot load failed'));
        img.src = snapUrl;
      });

      // Draw to canvas and run detection
      const canvas = document.getElementById('det-canvas');
      canvas.width  = snapImg.naturalWidth  || 320;
      canvas.height = snapImg.naturalHeight || 240;
      const ctx = canvas.getContext('2d');
      ctx.drawImage(snapImg, 0, 0, canvas.width, canvas.height);

      const preds = await detector.detect(canvas);
      const good  = preds.filter(p => p.score > 0.45);

      if(good.length > 0){
        // Sort by area (largest = closest / most prominent)
        good.sort((a, b) => (b.bbox[2] * b.bbox[3]) - (a.bbox[2] * a.bbox[3]));
        const label = good[0].class;

        document.getElementById('obj-name').textContent =
          good.map(p => p.class + ' ' + (p.score * 100).toFixed(0) + '%').join(' · ');
        document.getElementById('det-status').textContent =
          'Frame: ' + canvas.width + 'x' + canvas.height + ' · ' + good.length + ' object(s)';

        // Speak when ANY object detected — tighten threshold once confirmed working
        const now = Date.now();
        if(label !== lastDetLabel || now - lastDetSpeak > 5000){
          if(currentDist < 5){
            speak(label + ', danger, stop now!', true);
            fetch('/alert?level=3');
          } else if(currentDist < 50){
            speak(label + ' ahead, ' + Math.round(currentDist) + ' centimetres', false);
            fetch('/alert?level=1');
          } else {
            // Still speak even far away so you can verify detection works
            speak(label + ' detected', false);
          }
          lastDetSpeak = now;
          lastDetLabel = label;
        }
      } else {
        document.getElementById('obj-name').textContent = 'Clear path';
        document.getElementById('det-status').textContent =
          'Frame: ' + canvas.width + 'x' + canvas.height + ' · no objects';
        lastDetLabel = '';
      }

    } catch(e) {
      document.getElementById('obj-name').textContent = 'AI watching...';
      document.getElementById('det-status').textContent = 'Frame error: ' + e.message;
    }
  }

  detRunning = false;
  setTimeout(detectLoop, 1000); // Run every 1 second
}

// ── Obstacle & nav audio ──────────────────────────────────────
let lastObstacleSpeak = 0, lastNavSpeak = 0, lastNavDir = '', arrivedSpoken = false;

function checkObstacle(dist){
  const now = Date.now();
  if(dist < 5 && now - lastObstacleSpeak > 3000){
    speak('Danger! Stop immediately!', true);
    fetch('/alert?level=3');
    lastObstacleSpeak = now;
  }
}

function checkNav(nav){
  const now = Date.now();
  if(!nav || !nav.active){ arrivedSpoken = false; return; }
  if(nav.waiting){
    if(now - lastNavSpeak > 12000){
      speak('Waiting for GPS. Please go outdoors and allow location access.', false);
      lastNavSpeak = now;
    }
    return;
  }
  const dist = parseFloat(nav.distance);
  if(dist < 15){
    if(!arrivedSpoken){
      speak('You have arrived at ' + nav.target, true);
      arrivedSpoken = true;
      fetch('/nav?cmd=stop');
    }
    return;
  }
  arrivedSpoken = false;
  const dirChanged = (nav.turn !== lastNavDir);
  if(now - lastNavSpeak > 10000 || dirChanged){
    let msg = '';
    if(nav.turn === 'left')          msg = 'Turn left. ';
    else if(nav.turn === 'right')    msg = 'Turn right. ';
    else if(nav.turn === 'straight') msg = 'Continue straight. ';
    else                             msg = 'Head ' + nav.compass + '. ';
    msg += Math.round(dist) + ' metres to ' + nav.target + '.';
    speak(msg, dirChanged);
    lastNavSpeak = now;
    lastNavDir = nav.turn;
  }
}

// ── Status polling ────────────────────────────────────────────
function updateStatus(){
  fetch('/status')
    .then(r => r.json())
    .then(d => {
      currentDist = parseFloat(d.distance);

      const distEl = document.getElementById('dist');
      distEl.textContent = currentDist >= 999 ? '---' : Math.round(currentDist);
      distEl.className = 'val';
      if(currentDist < 5)       distEl.classList.add('danger');
      else if(currentDist < 20) distEl.classList.add('warn');

      checkObstacle(currentDist);

      const fixEl = document.getElementById('fix-val');
      fixEl.textContent = d.gps_fix ? '✅' : '❌';
      fixEl.style.color = d.gps_fix ? '#00ff88' : '#ff4444';

      const coordsEl = document.getElementById('coords');
      const mapLink  = document.getElementById('map-link');
      if(d.gps_fix && d.lat !== 0){
        coordsEl.textContent = d.lat.toFixed(6) + ', ' + d.lng.toFixed(6);
        mapLink.href = 'https://maps.google.com/?q=' + d.lat + ',' + d.lng;
        mapLink.style.display = 'block';
      } else {
        coordsEl.textContent = 'Searching — go outdoors for GPS fix';
        mapLink.style.display = 'none';
      }

      const nav    = d.nav;
      const navDiv = document.getElementById('nav-info');
      if(nav && nav.active && !nav.waiting){
        navDiv.innerHTML = '<b>' + nav.target + '</b> — ' + Math.round(nav.distance) + 'm<br>' +
          '🧭 Head ' + nav.compass + (nav.turn !== 'unknown' ? ' · 👉 ' + nav.turn.toUpperCase() : '');
        checkNav(nav);
      } else if(nav && nav.active && nav.waiting){
        navDiv.textContent = 'Navigating — waiting for GPS (go outdoors)';
        checkNav(nav);
      } else {
        navDiv.textContent = 'Not navigating';
      }

      document.getElementById('sbar').textContent =
        'Updated ' + new Date().toLocaleTimeString();
    })
    .catch(() => {
      document.getElementById('sbar').textContent = 'Connection lost — retrying...';
    });
}

// ── Boot ─────────────────────────────────────────────────────
window.addEventListener('load', () => {
  startPhoneGPS();
  loadDetector();   // loads model then starts detectLoop
  updateStatus();
  setInterval(updateStatus, 1500);
});
</script>
</body>
</html>
)rawhtml";
  server.send(200,"text/html",html);
}

// ============================================================
//  setup()
// ============================================================
void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Smart Cane Booting ===");

  pinMode(TRIG_PIN,OUTPUT); pinMode(ECHO_PIN,INPUT);
  pinMode(VIBRATION_PIN,OUTPUT); pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(TRIG_PIN,LOW); digitalWrite(VIBRATION_PIN,LOW); digitalWrite(BUZZER_PIN,LOW);

  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  int tries=0;
  while(WiFi.status()!=WL_CONNECTED && tries<40){ delay(500); Serial.print("."); tries++; }
  if(WiFi.status()==WL_CONNECTED){
    Serial.println("\nWiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    Serial.println(">>> Open IP in phone browser <<<");
  } else {
    Serial.println("\nWiFi FAILED — check SSID/password");
  }

  delay(200);
  cameraOK = setupCamera();
  if(cameraOK) startCameraServer();

  server.on("/",        handleRoot);
  server.on("/status",  handleStatus);
  server.on("/gps",     handleGpsUpdate);
  server.on("/nav",     handleNav);
  server.on("/alert",   handleAlert);
  server.on("/capture", handleCapture);   // ← NEW endpoint for AI detection

  server.begin();
  Serial.println("HTTP server started on port 80");
  Serial.println("=== Smart Cane Ready ===");
  beep(3,150);
  delay(100);
  vibrate(1,300);
}

// ============================================================
//  loop()
// ============================================================
void loop(){
  server.handleClient();

  unsigned long now = millis();
  if(now - lastDistanceMs >= 300){
    lastDistanceMs = now;
    currentDistance = measureDistanceCm();
    Serial.printf("Dist:%.1fcm | GPS:%.6f,%.6f | Fix:%s | Nav:%s\n",
      currentDistance, phoneLat, phoneLng,
      phoneGpsFix ? "YES" : "NO",
      navigating   ? SAVED_LOCATIONS[navTarget].name : "off");

    // Hardware alert ONLY when under 5 cm
    if(currentDistance < 5.0 && now - lastAlertMs > 2000){
      vibrate(3,300); beep(3,300); lastAlertMs = now;
    }
  }
}
