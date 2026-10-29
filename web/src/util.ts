export const uid = (n = 6): string =>
  Array.from({ length: n }, () => Math.floor(Math.random() * 16).toString(16)).join("");

export const linspace = (a: number, b: number, n: number): number[] =>
  Array.from({ length: n }, (_, i) => (n === 1 ? a : a + ((b - a) * i) / (n - 1)));
