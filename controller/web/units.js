/* Temperature display unit for every page: °C (default) or °F, a device-wide
   setting (Settings page, GET/POST /api/settings). Only what the pages show
   and take changes: the firmware, stored profiles, console and logs are
   always °C, so pages convert at the edges with fromC()/toC().

   The unit is read once at load. Telemetry carries the device's current unit;
   a page that sees it change reloads, so everything on it is redrawn in the
   new unit (reflowUnits.check(unit)). */
(function () {
  var unit = 'C';
  var readyCbs = [], loaded = false;

  function set(u) { unit = u === 'F' ? 'F' : 'C'; }
  function fromC(c) { return unit === 'F' ? c * 9 / 5 + 32 : c; }
  function toC(v) { return unit === 'F' ? (v - 32) * 5 / 9 : v; }
  /* Rates and differences scale but do not shift. */
  function deltaFromC(d) { return unit === 'F' ? d * 9 / 5 : d; }
  function symbol() { return unit === 'F' ? '°F' : '°C'; }
  /* "217 °C" or "422.6 °F"; digits = decimals (default 0). */
  function fmt(c, digits) { return fromC(c).toFixed(digits || 0) + ' ' + symbol(); }

  window.reflowUnits = {
    get unit() { return unit; },
    fromC: fromC, toC: toC, deltaFromC: deltaFromC, symbol: symbol, fmt: fmt,
    /* Run cb once the unit is known (immediately if it already is). */
    ready: function (cb) { if (loaded) cb(); else readyCbs.push(cb); },
    /* Telemetry reports the device's unit: reload if it changed elsewhere. */
    check: function (u) { if (loaded && u && u !== unit) location.reload(); }
  };

  fetch('/api/settings').then(function (r) { return r.json(); })
    .then(function (s) { set(s.temp_unit); })
    .catch(function () { /* keep °C */ })
    .then(function () {
      loaded = true;
      var cbs = readyCbs; readyCbs = [];
      for (var i = 0; i < cbs.length; i++) cbs[i]();
    });
})();
