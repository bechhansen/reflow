/* Light/dark theme for every page. It follows the browser's setting
   (prefers-color-scheme) unless the user picks the other one with a theme
   button; that choice is remembered in this browser, and picking what the
   browser prefers returns to following it. Loaded in <head>, before the page
   renders, so there is no flash of the wrong theme.

   Pages style themselves from CSS variables: light values on :root, dark
   values under :root[data-theme="dark"] and, while no choice is saved, under
   prefers-color-scheme: dark. Anything drawn in script (the chart) reads the
   same variables and subscribes with reflowTheme.onChange(). */
(function () {
  var KEY = 'reflow-theme';
  var root = document.documentElement;
  var mq = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;
  var listeners = [];

  function saved() {
    try { return localStorage.getItem(KEY); } catch (e) { return null; }
  }
  function browserTheme() { return mq && mq.matches ? 'dark' : 'light'; }
  function effective() { return root.getAttribute('data-theme') || browserTheme(); }

  function label() {
    var next = effective() === 'dark' ? 'light' : 'dark';
    var btns = document.querySelectorAll('.theme-btn');
    for (var i = 0; i < btns.length; i++) {
      /* Named by what it switches TO; text, not just an icon or a colour. */
      btns[i].textContent = next === 'dark' ? '☾ Dark' : '☀ Light';
      btns[i].setAttribute('aria-label', 'Switch to ' + next + ' theme');
      btns[i].title = 'Switch to ' + next + ' theme';
    }
  }

  function apply() {
    var s = saved();
    if (s === 'light' || s === 'dark') root.setAttribute('data-theme', s);
    else root.removeAttribute('data-theme');
    label();
    for (var i = 0; i < listeners.length; i++) listeners[i](effective());
  }

  function toggle() {
    var next = effective() === 'dark' ? 'light' : 'dark';
    try {
      if (next === browserTheme()) localStorage.removeItem(KEY);
      else localStorage.setItem(KEY, next);
    } catch (e) {
      /* No storage (private mode): switch for this page view only. */
      root.setAttribute('data-theme', next);
      label();
      for (var i = 0; i < listeners.length; i++) listeners[i](next);
      return;
    }
    apply();
  }

  if (mq) {
    if (mq.addEventListener) mq.addEventListener('change', apply);
    else if (mq.addListener) mq.addListener(apply);
  }
  window.reflowTheme = {
    effective: effective,
    toggle: toggle,
    onChange: function (f) { listeners.push(f); }
  };
  apply();
  document.addEventListener('DOMContentLoaded', label);
})();
