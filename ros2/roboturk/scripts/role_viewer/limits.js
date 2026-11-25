function createLimitControls(container, ranges, onChange) {
  const fields = {
    max_speed: ["Speed", "Robot top speed", "m/s"],
    max_omega: ["Turn", "Robot rotation limit", "rad/s"],
    max_acceleration: ["Accel", "Acceleration and braking", "m/s²"],
    max_kick: ["Kick", "Kick speed limit", "m/s"],
  };
  const controls = {};
  for (const [key, [name, title, unit]] of Object.entries(fields)) {
    const label = document.createElement("label");
    label.title = title;
    const caption = document.createElement("span");
    caption.textContent = name;
    const value = document.createElement("output");
    const slider = document.createElement("input");
    slider.type = "range";
    slider.id = `limit-${key}`;
    slider.step = "0.1";
    [slider.min, slider.max] = ranges[key];
    slider.disabled = true;
    slider.setAttribute("aria-label", title);
    const preview = () => {
      value.textContent = `${slider.value} ${unit}`;
      slider.setAttribute("aria-valuetext", value.textContent);
    };
    slider.addEventListener("input", preview);
    slider.addEventListener("change", () => onChange(key, slider.valueAsNumber));
    const heading = document.createElement("span");
    heading.className = "limit-heading";
    heading.append(caption, value);
    label.append(heading, slider);
    container.append(label);
    controls[key] = { label, slider, value, title, unit };
  }
  return {
    show(limits, editable, pending = new Map()) {
      container.hidden = !limits;
      if (!limits) return;
      for (const [key, { label, slider, value, title, unit }] of Object.entries(controls)) {
        const blue = pending.get(key) ?? limits.blue[key];
        const yellow = pending.get(key) ?? limits.yellow[key];
        const split = blue !== yellow;
        const display = blue == null ? "—" : `${split ? `${blue}/${yellow}` : blue} ${unit}`;
        label.title = split ? `${title} · Blue ${blue} / Yellow ${yellow} ${unit}` : title;
        slider.disabled = !editable;
        if (blue != null) slider.value = blue;
        value.textContent = display;
        slider.setAttribute("aria-valuetext", split ? label.title : display);
      }
    },
  };
}
