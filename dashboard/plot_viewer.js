// plot_viewer.js
// Called by index.html to load snapshot thumbnails or do other interactive tasks

function loadSnapshots(snapshotList) {
  const container = document.getElementById('snapshotGallery');
  container.innerHTML = ''; // clear

  snapshotList.forEach(snapPath => {
    const img = document.createElement('img');
    img.src = snapPath;
    img.className = 'snapshotThumb';
    img.title = snapPath;
    container.appendChild(img);
  });
}
