/* Embedded webpage served by the ESP SoftAP
 * Contains a joystick UI and a small WebSocket client that sends
 * joystick {x,y} JSON to ws://<device>/ws
 */

#ifndef WEB_PAGES_H
#define WEB_PAGES_H

static const char index_html[] =
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
      "<meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>TinyWhoop Joystick</title>"
      "<style>"
        "body{font-family:Arial,Helvetica,sans-serif;display:flex;flex-direction:column;align-items:center;justify-content:flex-start;padding:16px;background:#f5f5f5;color:#222}"
        ".joy-area{width:240px;height:240px;border-radius:50%;background:radial-gradient(circle at 30% 30%,#eee,#ddd);position:relative;touch-action:none;user-select:none;display:flex;align-items:center;justify-content:center;box-shadow:0 4px 10px rgba(0,0,0,0.15)}"
        ".knob{width:56px;height:56px;border-radius:50%;background:#ff6b6b;box-shadow:0 2px 6px rgba(0,0,0,0.3);position:absolute;transform:translate(-50%,-50%);left:50%;top:50%;transition:background 0.08s}"
        ".coords{margin-top:12px;font-size:14px}"
      "</style>"
    "</head>"
    "<body>"
      "<h1>Joystick</h1>"
      "<div id='joy' class='joy-area' role='application' aria-label='Joystick area'>"
        "<div id='knob' class='knob'></div>"
      "</div>"
      "<div class='coords'>X: <span id='x'>0.00</span> Y: <span id='y'>0.00</span></div>"
      "<script>"
        "(function(){"
          "const area = document.getElementById('joy');"
          "const knob = document.getElementById('knob');"
          "const xEl = document.getElementById('x');"
          "const yEl = document.getElementById('y');"
          "let active=false;"
          "let center={x:0,y:0};"
          "let ws = null;"
          "function rect(){return area.getBoundingClientRect();}"
          "function getMaxDist(){const r=rect(); return (r.width/2) - (knob.offsetWidth/2); }"
          "function updateCenter(){const r=rect(); center = {x: r.left + r.width/2, y: r.top + r.height/2}; }"
          "function initWS(){ try{ ws = new WebSocket('ws://' + location.host + '/ws'); ws.onopen = function(){ console.log('ws open'); }; ws.onmessage = function(e){ console.log('ws msg', e.data); }; ws.onclose = function(){ console.log('ws close'); setTimeout(initWS,1000); }; ws.onerror = function(){ /* ignore */ }; }catch(e){ setTimeout(initWS,1000); } }"
          "/* throttle sending to ~20Hz to avoid flooding the network */"
          "let _lastSend = 0;"
          "function sendToServer(nx,ny){ const now = Date.now(); if (now - _lastSend < 50) return; _lastSend = now; if(ws && ws.readyState===1){ try{ ws.send(JSON.stringify({x: nx, y: ny})); return; }catch(e){} } /* fallback to HTTP POST */ try{ fetch('/joy', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:nx,y:ny}), keepalive:true}); }catch(e){} }"
          "function handlePointer(evt){ evt.preventDefault(); const px = ('clientX' in evt) ? evt.clientX : (evt.touches && evt.touches[0] && evt.touches[0].clientX); const py = ('clientY' in evt) ? evt.clientY : (evt.touches && evt.touches[0] && evt.touches[0].clientY); if(px==null||py==null) return; const dx = px - center.x; const dy = py - center.y; const dist = Math.hypot(dx,dy); const angle = Math.atan2(dy,dx); const maxDist = getMaxDist() || 1; const limited = Math.min(dist, maxDist); const nx = Math.cos(angle)*limited; const ny = Math.sin(angle)*limited; const posX = (rect().width/2) + nx; const posY = (rect().height/2) + ny; knob.style.left = posX + 'px'; knob.style.top = posY + 'px'; const normX = (nx / maxDist); const normY = (ny / maxDist); xEl.textContent = normX.toFixed(2); yEl.textContent = normY.toFixed(2); sendToServer(normX, normY); }"
          "function start(e){ active=true; updateCenter(); if(e.pointerId && area.setPointerCapture) area.setPointerCapture(e.pointerId); handlePointer(e); knob.style.background='#ff4b4b'; }"
          "function move(e){ if(!active) return; handlePointer(e); }"
          "function end(e){ active=false; const r=rect(); knob.style.left = (r.width/2) + 'px'; knob.style.top = (r.height/2) + 'px'; xEl.textContent='0.00'; yEl.textContent='0.00'; sendToServer(0,0); knob.style.background='#ff6b6b'; }"
          "area.addEventListener('pointerdown', start);"
          "area.addEventListener('pointermove', move);"
          "area.addEventListener('pointerup', end);"
          "area.addEventListener('pointercancel', end);"
          "window.addEventListener('resize', updateCenter);"
          "window.addEventListener('load', function(){ const r=rect(); knob.style.left = (r.width/2) + 'px'; knob.style.top = (r.height/2) + 'px'; updateCenter(); initWS(); });"
        "})();"
      "</script>"
    "</body>"
    "</html>";

#endif /* WEB_PAGES_H */
