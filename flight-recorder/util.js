function hex(s) {
	return parseInt(s, 16);
}

function addToPath(lat, lng, path) {
	const current = L.latLng(lat, lng);
	let addAsNext;
	if (!path.isEmpty()) {
		const points = path.getLatLngs();
		const prev = points[points.length-1];
		if (current.distanceTo(prev) > 1.0) {
			addAsNext = true;
		}
	} else {
		addAsNext = true;
	}
	if (addAsNext) {
		path.addLatLng(current);
		return current;
	}
	return null;
}

function addAltitudeMarker(alt, path, altMarkers) {
	if (!path.isEmpty()) {
		const points = path.getLatLngs();
		const current = points[points.length-1];
		if (!('alt' in current)) {
			current.alt = alt;
			let prevAlt;
			if (altMarkers.length > 0) {
				prevAlt = altMarkers[altMarkers.length-1].getLatLng().alt;
			} else {
				prevAlt = 0.0;
			}
			if (Math.abs(alt - prevAlt) > 15.0) {
				// m to ft
				const altLabel = Math.round(current.alt*3.2808399);
				const altMarker = L.marker(current, {icon: L.divIcon({html: altLabel, iconSize: [30,12], className: 'altitude-marker'})});
				altMarker.addTo(map);
				altMarkers.push(altMarker);
				return altMarker;
			}
		}
	}
	return null;
}