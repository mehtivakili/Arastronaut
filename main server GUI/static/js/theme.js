// theme.js
function toggleTheme() {
    if (document.body.classList.contains('light-theme')) {
        document.body.classList.remove('light-theme');
        localStorage.setItem('theme', 'dark');
    } else {
        document.body.classList.add('light-theme');
        localStorage.setItem('theme', 'light');
    }
}

function applyTheme() {
    const theme = localStorage.getItem('theme');
    if (theme === 'light') {
        document.body.classList.add('light-theme');
    } else {
        document.body.classList.remove('light-theme');
    }
}

// Apply the theme when the document is loaded
document.addEventListener('DOMContentLoaded', applyTheme);
