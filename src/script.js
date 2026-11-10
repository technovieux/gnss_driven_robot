(() => {
    const points = [];
    const checkpoints = [];
    const origin = { lat: 47.480718234114995, lon: -0.6024181548374733 };
    let position = { ...origin };
    let heading = 0;
    let altitude = null;
    let manual = false;
    let progMode = false;
    let gnssSocket = null;
    let validPositionCount = 0;
    let lastGnssPointKey = "";

    const csvRows = document.getElementById('csvRows');
    const emptyRows = document.getElementById('emptyRows');
    const cpRows = document.getElementById('cpRows');
    const emptyCpRows = document.getElementById('emptyCpRows');
    const mapCanvas = document.getElementById('mapCanvas');

    const leafletMap = L.map(mapCanvas, { zoomControl: false }).setView([position.lat, position.lon], 17);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(leafletMap);
    L.control.zoom({ position: 'topright' }).addTo(leafletMap);

    const routeCoordinates = [[position.lat, position.lon]];
    const routeLine = L.polyline(routeCoordinates, { color: '#39845a', weight: 3, opacity: .9 }).addTo(leafletMap);
    const cpLayer = L.layerGroup().addTo(leafletMap);

    const sensorIcon = L.divIcon({
        className: 'sensor-marker-wrapper',
        html: '<div class="sensor-marker"><svg viewBox="0 0 32 32" fill="none"><circle cx="16" cy="16" r="14" fill="#ffffff" fill-opacity=".92"/><path d="M16 5 23 23l-7-4-7 4L16 5Z" fill="#267349" stroke="#ffffff" stroke-width="1.2" stroke-linejoin="round"/></svg></div>',
        iconSize: [32, 32], iconAnchor: [16, 16]
    });
    const sensorMarker = L.marker([position.lat, position.lon], { icon: sensorIcon, interactive: false }).addTo(leafletMap);

    function updateMap() {
        sensorMarker.setLatLng([position.lat, position.lon]);
        routeLine.setLatLngs(routeCoordinates);
    }

    // Interpolation par spline de Catmull-Rom pour la trajectoire optimale en courbe
    function getCatmullRomSplinePoints(pts, numPoints = 20) {
        if (pts.length < 2) return pts.map(p => [p.lat, p.lon]);
        
        const extended = [pts[0], ...pts, pts[pts.length - 1]];
        const curve = [];

        for (let i = 0; i < extended.length - 3; i++) {
            const p0 = extended[i];
            const p1 = extended[i + 1];
            const p2 = extended[i + 2];
            const p3 = extended[i + 3];

            for (let j = 0; j <= numPoints; j++) {
                if (i < extended.length - 4 && j === numPoints) continue;
                const t = j / numPoints;
                const tt = t * t;
                const ttt = tt * t;

                const qx = 0.5 * ((2 * p1.lat) + (-p0.lat + p2.lat) * t + (2 * p0.lat - 5 * p1.lat + 4 * p2.lat - p3.lat) * tt + (-p0.lat + 3 * p1.lat - 3 * p2.lat + p3.lat) * ttt);
                const qy = 0.5 * ((2 * p1.lon) + (-p0.lon + p2.lon) * t + (2 * p0.lon - 5 * p1.lon + 4 * p2.lon - p3.lon) * tt + (-p0.lon + 3 * p1.lon - 3 * p2.lon + p3.lon) * ttt);

                curve.push([qx, qy]);
            }
        }
        return curve;
    }

    let cpSplineLine = null;

    function renderCheckpoints() {
        console.log("[PROG] Rendu des checkpoints, total :", checkpoints.length);
        cpRows.innerHTML = '';
        cpLayer.clearLayers();
        
        if (emptyCpRows) {
            emptyCpRows.style.display = checkpoints.length > 0 ? 'none' : 'block';
        }

        checkpoints.forEach((cp) => {
            // Ajout dans le tableau HTML
            const row = document.createElement('tr');
            const badgeClass = cp.reached ? 'badge-reached' : 'badge-pending';
            const badgeText = cp.reached ? 'Atteint' : 'En attente';
            row.innerHTML = `<td>#${cp.id}</td><td>${cp.lat.toFixed(6)}</td><td>${cp.lon.toFixed(6)}</td><td><span class="badge-status ${badgeClass}">${badgeText}</span></td>`;
            cpRows.appendChild(row);

            // Ajout du marqueur sur la carte Leaflet
            const markerColor = cp.reached ? '#888888' : '#27734b';
            L.circleMarker([cp.lat, cp.lon], { 
                radius: 7, 
                color: '#ffffff', 
                weight: 2, 
                fillColor: markerColor, 
                fillOpacity: 1 
            }).bindPopup(`Checkpoint #${cp.id}<br>Statut: ${badgeText}`).addTo(cpLayer);
        });

        // Tracé de la spline si on a au moins 2 points
        if (checkpoints.length >= 2) {
            const smoothCoords = getCatmullRomSplinePoints(checkpoints, 25);
            cpSplineLine = L.polyline(smoothCoords, {
                color: '#d2913b',
                weight: 3.5,
                opacity: 0.85,
                dashArray: '4, 4'
            }).addTo(cpLayer); // On l'ajoute directement au cpLayer pour un nettoyage facile
        }
    }

    // Gestion du mode PROG
    const progButton = document.getElementById('progButton');
    progButton.addEventListener('click', () => {
        progMode = !progMode;
        progButton.classList.toggle('active', progMode);
        if (progMode) {
            if (gnssSocket && gnssSocket.readyState === WebSocket.OPEN) {
                gnssSocket.send(JSON.stringify({ action: "manual_move", direction: "STOP" }));
                gnssSocket.send(JSON.stringify({ action: "clear_route" }));
            }
            progButton.style.background = '#d8f2dc';
            document.getElementById('controlCaption').textContent = "Mode PROG actif · Cliquez sur la carte pour ajouter des checkpoints";
        } else {
            progButton.style.background = '';
            document.getElementById('controlCaption').textContent = "Mode automatique";
        }
    });

    leafletMap.on('click', (e) => {
        console.log("[MAP] Clic détecté sur la carte, progMode =", progMode);
        if (!progMode) return;
        
        const newCp = { 
            id: checkpoints.length + 1, 
            lat: e.latlng.lat, 
            lon: e.latlng.lng, 
            reached: false 
        };
        checkpoints.push(newCp);
        renderCheckpoints();

        if (gnssSocket && gnssSocket.readyState === WebSocket.OPEN) {
            gnssSocket.send(JSON.stringify({ action: "add_checkpoint", lat: newCp.lat, lng: newCp.lon }));
        }
    });

    document.getElementById('clearCheckpoints').addEventListener('click', () => {
        checkpoints.length = 0;
        renderCheckpoints();
        if (gnssSocket && gnssSocket.readyState === WebSocket.OPEN) {
            gnssSocket.send(JSON.stringify({ action: "clear_route" }));
        }
    });

    function addPoint() {
        const point = { id: points.length + 1, time: new Date(), lat: position.lat, lon: position.lon, altitude };
        points.unshift(point);
        if (points.length > 100) points.pop();
        const row = document.createElement('tr');
        row.innerHTML = `<td>${point.id}</td><td>${point.time.toLocaleTimeString('fr-FR')}</td><td>${point.lat.toFixed(6)}</td><td>${point.lon.toFixed(6)}</td><td>${Number.isFinite(point.altitude) ? point.altitude.toFixed(1) : '—'}</td>`;
        csvRows.prepend(row);
        while (csvRows.children.length > 100) csvRows.lastElementChild.remove();
        emptyRows.hidden = true;
        document.getElementById('pointCount').textContent = String(points.length).padStart(2, '0');
        
        // Vérification du rayon d'atteinte de 1,50 m pour valider le checkpoint
        checkpoints.forEach(cp => {
            if (!cp.reached) {
                const dLat = (position.lat - cp.lat) * 111320;
                const dLon = (position.lon - cp.lon) * 111320 * Math.cos(cp.lat * Math.PI / 180);
                const dist = Math.hypot(dLat, dLon);
                if (dist <= 1.5) {
                    cp.reached = true;
                    renderCheckpoints();
                }
            }
        });
    }

    function setGnssStatus(online, message) {
        document.getElementById('gnssStatusText').textContent = message;
        document.getElementById('gnssStatus').classList.toggle('offline', !online);
        document.getElementById('gnssDot').style.background = online ? '#388c57' : '#d2913b';
    }

    function handleGnssMessage(message) {
        if (!message) return;
        if (message.type === 'system_stats') {
            const cpuPercent = Number(message.cpu_percent);
            const ramPercent = Number(message.ram_percent);
            if (Number.isFinite(cpuPercent) && cpuPercent >= 0 && cpuPercent <= 100) {
                document.getElementById('cpuPercent').textContent = cpuPercent.toFixed(1);
            }
            if (Number.isFinite(ramPercent) && ramPercent >= 0 && ramPercent <= 100) {
                document.getElementById('ramPercent').textContent = ramPercent.toFixed(1);
            }
            return;
        }
        if (message.type !== 'gnss_update') return;
        const lat = Number(message.lat);
        const lon = Number(message.lng);
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

        validPositionCount++;
        position = { lat, lon };
        if (Number.isFinite(Number(message.altitude))) altitude = Number(message.altitude);

        document.getElementById('mapCoordinate').textContent = `${lat.toFixed(6)}° N · ${lon.toFixed(6)}° E`;
        
        if (!progMode) {
            const key = `${lat.toFixed(7)},${lon.toFixed(7)}`;
            if (key !== lastGnssPointKey) {
                lastGnssPointKey = key;
                routeCoordinates.push([lat, lon]);
                addPoint();
            }
            updateMap();
        }
        setGnssStatus(true, `GNSS · Mode ${message.mode}`);
    }

    function connectGnssWebSocket() {
        const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${scheme}//${window.location.hostname || 'localhost'}:9002`;
        gnssSocket = new WebSocket(url, 'robot-protocol');
        gnssSocket.onmessage = event => {
            try { handleGnssMessage(JSON.parse(event.data)); } catch(e){}
        };
        gnssSocket.onclose = () => setTimeout(connectGnssWebSocket, 2000);
    }

    document.getElementById('manualMode').addEventListener('change', event => {
        manual = event.target.checked;
        document.querySelectorAll('.direction').forEach(b => b.disabled = !manual);
    });

    document.querySelectorAll('.direction').forEach(button => {
        const dir = button.dataset.direction;
        const mapped = dir === 'up' ? 'AV' : dir === 'down' ? 'AR' : dir === 'left' ? 'G' : 'D';
        button.addEventListener('mousedown', () => manual && gnssSocket.send(JSON.stringify({action:"manual_move", direction:mapped})));
        button.addEventListener('mouseup', () => manual && gnssSocket.send(JSON.stringify({action:"manual_move", direction:"STOP"})));
    });

    document.getElementById('centerMap').addEventListener('click', () => leafletMap.setView([position.lat, position.lon], 17));
    connectGnssWebSocket();
})();